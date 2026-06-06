"""C 모듈 평가 — 올바른 GT (시나리오당 위험차 1대 명시)"""
import json, numpy as np
from collections import defaultdict
def load(f): return json.load(open(f'{f}/outputs/test_tracks_v1.1.json',encoding='utf-8'))
def pv(d):
    tr=defaultdict(lambda:{'lo':[],'lane':[],'pos':[]})
    for fr in d['frames']:
        for v in fr.get('vehicles',[]):
            t=v['track_id']
            tr[t]['lo'].append(v['lateral_offset_m']);tr[t]['lane'].append(v['lane_id']);tr[t]['pos'].append(v['position_road_m'])
    return tr
TH={'weaving':0.30,'lane_change':1,'tail_gap':5.0}
def suspects_of(name):
    d=load(name);tr=pv(d)
    valid={t:s for t,s in tr.items() if len(s['lo'])>=10}
    tail=set()
    for fr in d['frames']:
        bl=defaultdict(list)
        for v in fr.get('vehicles',[]): bl[v['lane_id']].append((v['position_road_m'][1],v['track_id']))
        for ps in bl.values():
            ps.sort()
            for (y1,_),(y2,i2) in zip(ps,ps[1:]):
                if abs(y2-y1)<TH['tail_gap']: tail.add(i2)
    sus=set()
    for t,s in valid.items():
        lane=np.array(s['lane']);ch=int(np.sum(np.abs(np.diff(lane))>=1))
        if (ch==0 and np.std(s['lo'])>TH['weaving']) or ch>=TH['lane_change'] or t in tail:
            sus.add(t)
    return sus,set(valid.keys())

# 정답: 시나리오별 실제 위험차량 ID
GT_OFFENDERS={
    'carla_normal': set(),
    'carla_lane_weaving': {6},
    'carla_tailgating': {12},
    'carla_sudden_lane_change': {17},
}
TP=FP=FN=TN=0
print(f"{'시나리오':<26}{'위험차(정답)':<14}{'C가잡은차':<14}")
for name,gt in GT_OFFENDERS.items():
    sus,allv=suspects_of(name)
    print(f"{name:<26}{str(sorted(gt)):<14}{str(sorted(sus)):<14}")
    for t in allv:
        pred=t in sus; truth=t in gt
        if pred and truth: TP+=1
        elif pred and not truth: FP+=1
        elif not pred and truth: FN+=1
        else: TN+=1
prec=TP/(TP+FP) if TP+FP else 0
rec=TP/(TP+FN) if TP+FN else 0
f1=2*prec*rec/(prec+rec) if prec+rec else 0
print(f"\n혼동행렬: TP={TP} FP={FP} FN={FN} TN={TN}")
print(f"  Precision = {prec:.3f}  (의심이라 한 것 중 진짜 비율)")
print(f"  Recall    = {rec:.3f}  (실제 위험차 중 잡은 비율)")
print(f"  F1        = {f1:.3f}")
print(f"  정확도    = {(TP+TN)/(TP+FP+FN+TN):.3f}")
