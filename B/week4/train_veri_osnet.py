"""
VeRi-776으로 OSNet fine-tuning (커스텀 데이터셋 등록 버전).

이 버전의 torchreid에는 'veri' 데이터셋 클래스가 없으므로 (person ReID 전용),
VeRi를 커스텀 ImageDataset으로 직접 등록한다.

VeRi 파일명 형식: {vehicleID}_c{cameraID}_{timestamp}_{seq}.jpg
  예: 0543_c009_00000135_0.jpg -> pid=543, camid=9

출력: log/osnet_veri/model/model.pth.tar-60
"""
import os
import sys
import glob
import re

import torch
import torchreid


HERE = os.path.dirname(os.path.abspath(__file__))
VERI_DIR = os.path.join(HERE, 'archive', 'VeRi')


def parse_veri(folder):
    """VeRi 이미지 폴더 -> [(img_path, pid, camid), ...]."""
    pattern = re.compile(r'(\d+)_c(\d+)')
    data = []
    for img_path in glob.glob(os.path.join(folder, '*.jpg')):
        fname = os.path.basename(img_path)
        m = pattern.match(fname)
        if not m:
            continue
        pid = int(m.group(1))
        camid = int(m.group(2))
        data.append((img_path, pid, camid))
    return data


class VeRiDataset(torchreid.data.ImageDataset):
    """VeRi-776 커스텀 데이터셋."""
    dataset_dir = ''

    def __init__(self, root='', **kwargs):
        train_dir = os.path.join(VERI_DIR, 'image_train')
        query_dir = os.path.join(VERI_DIR, 'image_query')
        gallery_dir = os.path.join(VERI_DIR, 'image_test')

        for d in (train_dir, query_dir, gallery_dir):
            if not os.path.isdir(d):
                raise RuntimeError('VeRi 폴더 없음: ' + d)

        train_raw = parse_veri(train_dir)
        query = parse_veri(query_dir)
        gallery = parse_veri(gallery_dir)

        # train pid를 0부터 연속 라벨로 인코딩 (torchreid 요구사항)
        pid_set = sorted({pid for _, pid, _ in train_raw})
        pid2label = {pid: idx for idx, pid in enumerate(pid_set)}
        train = [(p, pid2label[pid], cam) for p, pid, cam in train_raw]

        super(VeRiDataset, self).__init__(train, query, gallery, **kwargs)


def main():
    if not os.path.isdir(os.path.join(VERI_DIR, 'image_train')):
        print('[오류] VeRi 데이터 없음: ' + VERI_DIR + '\\image_train')
        sys.exit(1)

    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print('=== VeRi OSNet fine-tuning (커스텀 데이터셋) ===')
    print('Device:', device)
    print('VeRi:', VERI_DIR)

    torchreid.data.register_image_dataset('veri', VeRiDataset)

    datamanager = torchreid.data.ImageDataManager(
        root='',
        sources='veri',
        targets='veri',
        height=256,
        width=256,
        batch_size_train=32,
        batch_size_test=64,
        transforms=['random_flip', 'random_crop'],
    )

    print('학습 차량 ID 수:', datamanager.num_train_pids)

    model = torchreid.models.build_model(
        name='osnet_x1_0',
        num_classes=datamanager.num_train_pids,
        loss='softmax',
        pretrained=True,
    )
    model = model.to(device)

    optimizer = torchreid.optim.build_optimizer(model, optim='adam', lr=0.0015)
    scheduler = torchreid.optim.build_lr_scheduler(
        optimizer, lr_scheduler='cosine', max_epoch=60)

    engine = torchreid.engine.ImageSoftmaxEngine(
        datamanager, model, optimizer=optimizer, scheduler=scheduler,
        label_smooth=True)

    engine.run(
        save_dir='log/osnet_veri',
        max_epoch=60,
        eval_freq=20,
        print_freq=50,
        test_only=False,
    )

    print('\n=== 완료 ===')
    print('학습된 가중치: log/osnet_veri/model/model.pth.tar-60')


if __name__ == '__main__':
    main()
