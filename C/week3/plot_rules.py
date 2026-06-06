import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from collections import defaultdict
plt.rcParams['axes.unicode_minus']=False

def load(f): return json.load(open(f'{f}/outputs/test_tracks_v1.1.json',encoding='utf-8'))
def trajectories(d):
    tr=defaultdict(lambda:{'lo':[],'sp':[],'lane':[],'pos':[]})
    for fr in d['frames']:
        for v in fr.get('vehicles',[]):
            t=v['track_id']
            tr[t]['lo'].append(v['lateral_offset_m']); tr[t]['sp'].append(v['speed_est_mps'])
            tr[t]['lane'].append(v['lane_id']); tr[t]['pos'].append(v['position_road_m'])
    return tr
def r_weav(tr):
    v=[np.std(s['lo']) for s in tr.values() if len(s['lo'])>=10 and np.sum(np.abs(np.diff(s['lane']))>=1)==0]
    return np.mean(sorted(v)[-3:]) if v else 0
def r_speed(tr):
    m=[np.mean(np.array(s['sp'])[np.array(s['sp'])>0.1])*3.6 for s in tr.values() if np.sum(np.array(s['sp'])>0.1)>=5]
    return np.median(m) if m else 0
def r_lc(tr):
    c=[np.sum(np.abs(np.diff(s['lane']))>=1) for s in tr.values() if len(s['lane'])>=10]
    return max(c) if c else 0
def r_tg(d):
    mg=999
    for fr in d['frames']:
        bl=defaultdict(list)
        for v in fr.get('vehicles',[]): bl[v['lane_id']].append(v['position_road_m'])
        for ps in bl.values():
            if len(ps)<2: continue
            ys=sorted(p[1] for p in ps)
            for a,b in zip(ys,ys[1:]): mg=min(mg,abs(b-a))
    return mg if mg<999 else 0

SCN=['carla_normal','carla_lane_weaving','carla_tailgating','carla_sudden_lane_change','carla_speeding']
short=['normal','weaving','tailgating','lane_change','speeding']
R={f:(0,0,0,0) for f in SCN}
for f in SCN:
    d=load(f);tr=trajectories(d); R[f]=(r_weav(tr),r_speed(tr),r_lc(tr),r_tg(d))

fig,axes=plt.subplots(2,2,figsize=(13,9))
metrics=[('Lane Weaving (std, m)',0,'weaving','#cc3333'),
         ('Speeding (median km/h)',1,'speeding','#cc8800'),
         ('Lane Changes (count)',2,'lane_change','#3366cc'),
         ('Tailgating (min gap, m)',3,'tailgating','#008800')]
for ax,(title,idx,target,col) in zip(axes.flat,metrics):
    vals=[R[f][idx] for f in SCN]
    colors=[col if s==target else '#cccccc' for s in short]
    ax.bar(short,vals,color=colors)
    ax.set_title(title,fontsize=12,fontweight='bold')
    ax.tick_params(axis='x',rotation=30,labelsize=9)
    for i,v in enumerate(vals):
        ax.text(i,v,f'{v:.2f}' if idx!=2 else f'{int(v)}',ha='center',va='bottom',fontsize=9)
    note='(lower=worse)' if idx==3 else '(higher=worse)'
    ax.set_xlabel(f'colored = target scenario  {note}',fontsize=8)
plt.suptitle('C Module: 4 Behavior Rules — each peaks at its target scenario',fontsize=14,fontweight='bold')
plt.tight_layout()
plt.savefig('/mnt/user-data/outputs/C_4rules_validation.png',dpi=130,bbox_inches='tight')
print('saved')
