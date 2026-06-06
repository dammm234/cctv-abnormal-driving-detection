"""
ReID 모듈 - Step 1: Feature 추출

CARLA 8 시나리오 × 3 카메라의 각 차량 bbox crop에서 ReID feature 추출.

입력:
- data/scenarios/{시나리오}/cam{0,1,2}/*.png  (CARLA 프레임)
- scenarios_v1.1/{시나리오}_cam{0,1,2}.json   (bbox 정보)

출력:
- reid_features/{시나리오}/cam{i}_tracklet_features.npz
  각 tracklet의 평균 feature vector (512-dim)

사용 모델:
- 1순위: torchreid의 OSNet (Zhou et al. ICCV 2019)
- 2순위 (fallback): torchvision의 ResNet50 + GAP

근거:
- OSNet [16]: 차량 ReID에서 강력한 다중 스케일 특징
- VeRi-776 데이터셋으로 사전 학습 가중치 사용 가능
"""
import argparse
import json
import os
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

try:
    import torch
    import torch.nn as nn
    from torchvision import transforms, models
    from PIL import Image
except ImportError:
    print("[오류] torch/torchvision/Pillow 필요:")
    print("  pip install torch torchvision Pillow")
    sys.exit(1)


# 글로벌 설정
DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
FEATURE_DIM = 2048  # ResNet50 마지막 layer 출력


def get_reid_model(veri_weights=None):
    """ReID 모델 로드. OSNet 시도 실패 시 ResNet50 fallback.

    veri_weights: VeRi-776 fine-tuned 가중치 경로(.pth.tar).
        주어지면 ImageNet 대신 이 가중치를 로드 (차량 전용 OSNet).
        None이면 기존처럼 ImageNet 사전학습 OSNet (비교 기준군).
    """
    # Step 1: torchreid 시도
    try:
        import torchreid
        model = torchreid.models.build_model(
            name='osnet_x1_0',
            num_classes=576 if veri_weights else 1000,  # VeRi train = 576 차량
            loss='softmax',
            pretrained=(veri_weights is None),  # VeRi 쓰면 ImageNet 사전학습 불필요
        )

        if veri_weights:
            if not os.path.exists(veri_weights):
                raise FileNotFoundError(f'VeRi 가중치 없음: {veri_weights}')
            # import 경로가 torchreid 버전마다 달라 둘 다 시도
            try:
                from torchreid.reid.utils import load_pretrained_weights
            except ImportError:
                from torchreid.utils import load_pretrained_weights
            # feature(512-dim)만 사용하므로 분류층 크기 불일치는 자동 무시됨
            load_pretrained_weights(model, veri_weights)
            print(f'  ReID 모델: OSNet (VeRi-776 fine-tuned) on {DEVICE}')
            print(f'    가중치: {veri_weights}')
        else:
            print(f'  ReID 모델: OSNet (ImageNet) on {DEVICE}')

        model.eval()
        return model.to(DEVICE), 'osnet', 512
    except ImportError:
        print('  [INFO] torchreid 미설치, ResNet50 fallback 사용')
    except Exception as e:
        print(f'  [WARN] OSNet 로드 실패: {e}, ResNet50 fallback')

    # Step 2: ResNet50 fallback (avg pool로 feature 추출)
    resnet = models.resnet50(weights=models.ResNet50_Weights.IMAGENET1K_V2)
    # 마지막 FC layer 제거 (avg pool 출력까지만)
    feature_extractor = nn.Sequential(*list(resnet.children())[:-1])
    feature_extractor.eval()
    print(f'  ReID 모델: ResNet50 (ImageNet) on {DEVICE}')
    return feature_extractor.to(DEVICE), 'resnet50', 2048


def get_preprocess(model_type):
    """모델별 전처리."""
    if model_type == 'osnet':
        return transforms.Compose([
            transforms.Resize((256, 128)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                  std=[0.229, 0.224, 0.225]),
        ])
    else:  # resnet50
        return transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                  std=[0.229, 0.224, 0.225]),
        ])


def extract_features_from_crops(crops, model, preprocess, batch_size=16):
    """bbox crop 리스트 → feature vectors."""
    if not crops:
        return np.array([])

    tensors = []
    for crop in crops:
        try:
            t = preprocess(crop)
            tensors.append(t)
        except Exception as e:
            print(f'    crop 처리 실패: {e}')
            continue

    if not tensors:
        return np.array([])

    features = []
    with torch.no_grad():
        for i in range(0, len(tensors), batch_size):
            batch = torch.stack(tensors[i:i + batch_size]).to(DEVICE)
            feat = model(batch)
            if feat.dim() > 2:
                feat = feat.flatten(1)
            features.append(feat.cpu().numpy())

    return np.concatenate(features, axis=0)


def process_camera(scenario_dir, camera, json_path, model, preprocess,
                    sample_every=5):
    """1개 카메라의 모든 tracklet에서 평균 feature 추출."""
    # JSON 로드
    with open(json_path, encoding='utf-8') as f:
        data = json.load(f)

    fps = data.get('fps', 20.0)

    # frame_id → vehicles 매핑
    frame_vehicles = {}
    for frame in data['frames']:
        if frame.get('vehicles'):
            frame_vehicles[frame['frame_id']] = frame['vehicles']

    if not frame_vehicles:
        print(f'    [{camera}] vehicles 없음, skip')
        return {}

    # tracklet별 crop 수집 (sample_every로 다운샘플)
    track_crops = defaultdict(list)

    frame_ids = sorted(frame_vehicles.keys())
    for fid in frame_ids[::sample_every]:  # 5프레임마다 1개 샘플링
        png_path = os.path.join(scenario_dir, camera, f'{fid:06d}.png')
        if not os.path.exists(png_path):
            continue

        try:
            img = Image.open(png_path).convert('RGB')
        except Exception:
            continue

        for vehicle in frame_vehicles[fid]:
            track_id = vehicle['track_id']
            x1, y1, x2, y2 = vehicle['bbox_pixel']
            # 패딩 추가 (ReID는 차량 전체 외형 활용)
            pad = 5
            x1 = max(0, int(x1) - pad)
            y1 = max(0, int(y1) - pad)
            x2 = min(img.width, int(x2) + pad)
            y2 = min(img.height, int(y2) + pad)

            if x2 - x1 < 10 or y2 - y1 < 10:
                continue

            try:
                crop = img.crop((x1, y1, x2, y2))
                track_crops[track_id].append(crop)
            except Exception:
                continue

    # feature 추출 + tracklet별 평균
    tracklet_features = {}
    for track_id, crops in track_crops.items():
        if len(crops) < 2:  # 너무 적으면 신뢰 못함
            continue
        feats = extract_features_from_crops(crops, model, preprocess)
        if len(feats) == 0:
            continue
        # L2 정규화 후 평균 (코사인 유사도 효과)
        feats_normalized = feats / (np.linalg.norm(feats, axis=1, keepdims=True)
                                      + 1e-8)
        avg_feat = np.mean(feats_normalized, axis=0)
        avg_feat = avg_feat / (np.linalg.norm(avg_feat) + 1e-8)  # 재정규화

        tracklet_features[int(track_id)] = {
            'feature': avg_feat,
            'n_samples': len(crops),
            'first_frame': min(frame_ids),  # 단순 근사
            'last_frame': max(frame_ids),
        }

    print(f'    [{camera}] {len(track_crops)} tracks → '
          f'{len(tracklet_features)} valid features')
    return tracklet_features


def main():
    parser = argparse.ArgumentParser(
        description='CARLA 시나리오에서 ReID feature 추출',
    )
    parser.add_argument(
        '--scenarios-dir', default='../../../data/scenarios',
        help='CARLA 시나리오 루트',
    )
    parser.add_argument(
        '--json-dir', default='scenarios_v1.1',
        help='v1.1 JSON 폴더',
    )
    parser.add_argument(
        '--output-dir', default='reid_features',
        help='feature 출력 폴더',
    )
    parser.add_argument(
        '--sample-every', type=int, default=5,
        help='몇 프레임마다 샘플링 (속도 vs 정확도)',
    )
    parser.add_argument(
        '--veri-weights', default=None,
        help='VeRi-776 fine-tuned 가중치 경로(.pth.tar). '
             '주면 ImageNet 대신 차량 전용 OSNet 사용. '
             '예: log/osnet_veri/model/model.pth.tar-60',
    )
    args = parser.parse_args()

    if not os.path.isdir(args.scenarios_dir):
        print(f'[오류] 시나리오 폴더 없음: {args.scenarios_dir}')
        sys.exit(1)
    if not os.path.isdir(args.json_dir):
        print(f'[오류] JSON 폴더 없음: {args.json_dir}')
        sys.exit(1)

    os.makedirs(args.output_dir, exist_ok=True)

    print('=== ReID Feature 추출 시작 ===')
    print(f'Device: {DEVICE}')

    print('\n1. ReID 모델 로드')
    model, model_type, feat_dim = get_reid_model(veri_weights=args.veri_weights)
    preprocess = get_preprocess(model_type)

    # 시나리오 발견
    scenarios = sorted([d for d in os.listdir(args.scenarios_dir)
                        if os.path.isdir(os.path.join(args.scenarios_dir, d))])
    print(f'\n2. 시나리오 {len(scenarios)}개 처리')

    cameras = ['cam0', 'cam1', 'cam2']

    for scenario in scenarios:
        print(f'\n[{scenario}]')
        scenario_dir = os.path.join(args.scenarios_dir, scenario)

        scenario_features = {}
        for camera in cameras:
            json_path = os.path.join(args.json_dir,
                                      f'{scenario}_{camera}.json')
            if not os.path.exists(json_path):
                print(f'    [{camera}] JSON 없음, skip')
                continue

            features = process_camera(scenario_dir, camera, json_path,
                                       model, preprocess,
                                       sample_every=args.sample_every)
            scenario_features[camera] = features

        # 저장
        output_path = os.path.join(args.output_dir, f'{scenario}.npz')
        save_dict = {
            'model_type': model_type,
            'feature_dim': feat_dim,
            'scenario': scenario,
        }
        for camera, feats in scenario_features.items():
            for track_id, info in feats.items():
                key = f'{camera}_track_{track_id}'
                save_dict[key + '_feature'] = info['feature']
                save_dict[key + '_n_samples'] = info['n_samples']

        np.savez(output_path, **save_dict)
        print(f'  → {output_path}')

    print('\n=== 완료 ===')


if __name__ == '__main__':
    main()
