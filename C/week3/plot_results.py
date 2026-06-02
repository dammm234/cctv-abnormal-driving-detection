import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from collections import defaultdict

# 한글 폰트 시도 (없으면 영문 라벨 fallback)
KO = False
for f in ['NanumGothic','Noto Sans CJK KR','Malgun Gothic']:
    if any(f.lower() in fn.name.lower() for fn in font_manager.fontManager.ttflist):
        plt.rcParams['font.family']=f; KO=True; break
plt.rcParams['axes.unicode_minus']=False

def weaving(lo): return float(np.std(lo)) if len(lo)>=2 else 0.0
SCN='../../B/week3/scenarios_v1.1'

# --- 그래프 1: B 단조 증가 ---
grp=['wobble_mild','wobble_medium','wobble_strong']
wv=[]
for name in grp:
    d=json.load(open(f'{SCN}/{name}_cam2.json',encoding='utf-8'))
    lo=[fr['vehicles'][0]['lateral_offset_m'] for fr in d['frames'] if fr.get('vehicles')]
    wv.append(weaving(lo))
nrm=[]
for name in ['normal_50kmh','normal_60kmh']:
    d=json.load(open(f'{SCN}/{name}_cam2.json',encoding='utf-8'))
    lo=[fr['vehicles'][0]['lateral_offset_m'] for fr in d['frames'] if fr.get('vehicles')]
    nrm.append(weaving(lo))

fig,(ax1,ax2)=plt.subplots(1,2,figsize=(13,5))

L = ['normal','mild','medium','strong'] if KO else ['normal','mild','medium','strong']
bars=ax1.bar(L,[np.mean(nrm)]+wv,color=['#888','#aaffaa','#44dd44','#008800'])
t1='B 시나리오: weaving 단조 증가 (검증)' if KO else 'B scenarios: weaving monotonic increase'
ax1.set_title(t1,fontsize=13,fontweight='bold')
ax1.set_ylabel('weaving score (lateral_offset std, m)')
ax1.axhline(0.30,ls='--',color='red',alpha=0.6,label='threshold 0.30')
ax1.legend()
for b,v in zip(bars,[np.mean(nrm)]+wv):
    ax1.text(b.get_x()+b.get_width()/2,v+0.02,f'{v:.2f}',ha='center',fontsize=10)

# --- 그래프 2: A 실제 차량 분포 ---
d=json.load(open('../../A/week2/test_tracks_v1.1.json',encoding='utf-8'))
tr=defaultdict(list)
for fr in d['frames']:
    for v in fr.get('vehicles',[]):
        tr[v['track_id']].append(v['lateral_offset_m'])
scores=sorted([(t,weaving(lo)) for t,lo in tr.items() if len(lo)>=10],key=lambda x:-x[1])
ids=[str(t) for t,_ in scores]; vals=[v for _,v in scores]
colors=['#cc3333' if v>0.30 else '#88aacc' for v in vals]
ax2.bar(range(len(vals)),vals,color=colors)
ax2.axhline(0.30,ls='--',color='red',alpha=0.6,label='threshold 0.30')
t2='A 실제 CCTV: 차량별 weaving (빨강=의심 14대)' if KO else 'A real CCTV: per-vehicle weaving (red=flagged 14)'
ax2.set_title(t2,fontsize=13,fontweight='bold')
ax2.set_ylabel('weaving score (m)')
ax2.set_xlabel('vehicles (sorted)')
ax2.legend()

plt.tight_layout()
plt.savefig('/mnt/user-data/outputs/C_metric_results.png',dpi=130,bbox_inches='tight')
print('saved. 한글폰트:',KO)
