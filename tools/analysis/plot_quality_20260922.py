from pathlib import Path
import csv,json,math,sys
base=Path(__file__).resolve().parent.parent
if sys.platform=='win32' and sys.version_info[:2]==(3,12):sys.path.insert(0,str(base/'work/plot-libs'))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
font_manager.fontManager.addfont('C:/Windows/Fonts/msyh.ttc' if sys.platform=='win32' else '/mnt/c/Windows/Fonts/msyh.ttc')
plt.rcParams.update({'font.family':'Microsoft YaHei','axes.unicode_minus':False,'figure.dpi':150,'font.size':10})
out=base/'outputs/nav-quality-20260922';root=out/'nav-quality-20260922-pe7t26um'
report=json.loads((out/'analysis.json').read_text(encoding='utf-8'))
state=json.loads((root/'state.json').read_text(encoding='utf-8'))
rows=list(csv.DictReader((out/'trajectory.csv').open(encoding='utf-8')))
route=state['mission']['catalog']['full'];chain=[]
for i,p in enumerate(route['waypoints']):
    chain.append(p['xyz'])
    if i<len(route['edges']):chain.extend(c['xyz'] for c in route['edges'][i].get('control_points',[]))
chain=np.array(chain)
fig,axes=plt.subplots(2,2,figsize=(14,10))
ax=axes[0,0];ax.plot(chain[:,0],chain[:,1],color='#1758a1',lw=1.5,label='最终校准路线 · v103')
for owner,color,label in [('auto','#168567','自动导航定位轨迹'),('manual','#de8d27','人工接管定位轨迹')]:
    selected=[r for r in rows if r['owner']==owner]
    ax.scatter([float(r['x']) for r in selected],[float(r['y']) for r in selected],s=3,color=color,alpha=.5,label=label)
for name in ('WP 28','WP 48','WP 54'):
    p=next(p for p in route['waypoints'] if p['name']==name);ax.annotate(name,p['xyz'][:2],xytext=(8,7),textcoords='offset points')
ax.set_aspect('equal');ax.set_xlabel('地图 X / m');ax.set_ylabel('地图 Y / m');ax.legend(fontsize=9);ax.set_title('路线与定位估计（不作为外部精度真值）',loc='left')
t0=float(rows[0]['mono']);t=np.array([(float(r['mono'])-t0)/60 for r in rows])
ax=axes[0,1];ax.plot(t,[float(r['age_s']) for r in rows],color='#1758a1',lw=.8)
ax.axhline(.25,color='#d88426',ls='--',label='0.25 s');ax.set_xlabel('距日志开始 / min');ax.set_ylabel('测量年龄 / s');ax.set_title('Lightning 数据连续；主要问题是累计偏移',loc='left');ax.legend()
ax=axes[1,0]
for mode,label,color in [('FOLLOWING','正式路线跟随','#168567'),('JOINING_ROUTE','恢复后的路线接入','#de8d27')]:
    r=[r for r in rows if r['owner']=='auto' and r['state']==mode and r['cross_track']]
    ax.scatter([(float(v['mono'])-t0)/60 for v in r],[float(v['cross_track']) for v in r],s=7,color=color,label=label)
ax.set_xlabel('距日志开始 / min');ax.set_ylabel('横向误差 / m');ax.legend();ax.set_title('按当时的定位及执行路线计算的跟踪误差',loc='left')
ax=axes[1,1];matches=report['manual_alignment_changes'];x=np.arange(len(matches))
ax.bar(x-.16,[m['xy_change_m'] for m in matches],.32,label='水平修正',color='#1758a1')
ax.bar(x+.16,[abs(m['z_change_m']) for m in matches],.32,label='垂直修正',color='#de8d27')
ax.set_xticks(x,[m['time'][:8] for m in matches]);ax.set_ylabel('重定位前后位置变化 / m');ax.set_title('三次人工重定位揭示累计偏差',loc='left');ax.legend()
for ax in axes.flat:ax.grid(alpha=.18)
fig.suptitle('GOAI 路线修正与定位质量 · 2026-09-22\n可用日志 13:21:03—13:42:21；早期日志已轮转，曲线包含人工接管',fontweight='bold')
fig.tight_layout(rect=[0,0,1,.94]);fig.savefig(out/'quality-overview.png');plt.close(fig)
print(out/'quality-overview.png')
