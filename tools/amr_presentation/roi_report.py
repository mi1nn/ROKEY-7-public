#!/usr/bin/env python3
"""Summarize one ROI collection directory after capture; never mixes different trials."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import statistics


def number(row,key):
    value=row.get(key,'')
    return float(value) if value not in ('',None) else None


def summarize(folder, plots=True):
    folder=Path(folder)
    config=json.loads((folder/'trial.json').read_text())
    sizes=config['roi_sizes']
    with (folder/'samples.csv').open(newline='') as f: rows=list(csv.DictReader(f))
    pairs={}
    for row in rows:
        size=int(row['roi_size']); key=row['pair_id']
        if size not in sizes: raise ValueError('CSV size absent from trial config')
        group=pairs.setdefault(key,{})
        if size in group: raise ValueError(f'Duplicate pair/size: {key}/{size}')
        group[size]=row
    complete={k:g for k,g in pairs.items() if set(g)==set(sizes)}
    partial_count=len(pairs)-len(complete)
    common={k:g for k,g in complete.items() if all(r['status']=='LOCATED' and number(r,'z_m') is not None for r in g.values())}
    summary=[]
    for size in sizes:
        samples=[g[size] for g in complete.values()]
        passed=[r for r in samples if r['status']=='LOCATED' and number(r,'z_m') is not None]
        zs=[number(r,'z_m') for r in passed]
        errors=[number(r,'error_m') for r in passed if number(r,'error_m') is not None]
        common_errors=[number(g[size],'error_m') for g in common.values() if number(g[size],'error_m') is not None]
        reasons={}
        for row in samples: reasons[row['status']]=reasons.get(row['status'],0)+1
        mean=lambda x: statistics.mean(x) if x else None
        stats=dict(roi_size=size,matched_pairs=len(samples),passed_pairs=len(passed),
            pass_rate= len(passed)/len(samples) if samples else None,
            mean_z_m=mean(zs),std_z_m=statistics.stdev(zs) if len(zs)>=2 else None,
            bias_m=mean(errors),mae_m=mean([abs(e) for e in errors]),
            rmse_m=math.sqrt(mean([e*e for e in errors])) if errors else None,
            common_pass_pairs=len(common),common_mae_m=mean([abs(e) for e in common_errors]),
            common_bias_m=mean(common_errors),
            mean_eval_ms=mean([number(r,'compare_ms') for r in samples]),
            clipped_pairs=sum(r.get('clipped')=='True' for r in samples),reasons=reasons)
        summary.append(stats)
    end_path=folder/'collection_end.json'
    end=json.loads(end_path.read_text()) if end_path.exists() else None
    report=dict(trial_label=config['trial_label'],reference_z_m=config.get('reference_z_m'),
        status='NO_MATCHED_DATA' if not complete else ('INCOMPLETE_ROWS' if partial_count else 'SUMMARIZED'),
        complete_pairs=len(complete),incomplete_pairs_excluded=partial_count,
        collection_end=end,writer_count_matches=(end['accepted_pairs']==len(pairs)) if end else None,
        source_sha=config.get('source_sha'),metrics=summary,
        interpretation=['Pass denominator is matched eligible pairs, not all camera frames.',
            'Per-size MAE/std use accepted results only; also compare common_mae_m on the same accepted pairs.',
            'std_z_m describes temporal variation; interpret as noise only for a fixed camera and stationary surface.',
            'XYZ/Z uses configured depth correction; reference must be optical-axis surface Z.',
            'No valid reference: accuracy metrics remain empty. No pass is not zero error.',
            'Nominal ROI can be clipped by bbox/image; see per-frame roi_pixels and clipped.',
            'No automatic best-size verdict; report pass rate and errors together.'])
    if complete and (end is None or not report['writer_count_matches'] or end.get('dropped_pairs',0)>0):
        report['status']='PARTIAL_COLLECTION'
    (folder/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    fields=[k for k in summary[0] if k!='reasons']
    with (folder/'summary.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore'); writer.writeheader(); writer.writerows(summary)
    if plots: plot(folder,complete,summary,config)
    return report


def plot(folder,pairs,summary,config):
    os.environ.setdefault('MPLCONFIGDIR',str(folder/'plot_cache'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    fig,axs=plt.subplots(2,2,figsize=(13,8),constrained_layout=True)
    fig.suptitle('Center ROI comparison | '+config['trial_label'],fontsize=17)
    colors=['#0072B2','#D55E00','#009E73','#CC79A7','#E69F00']
    for i,item in enumerate(summary):
        size=item['roi_size']; data=sorted((g[size] for g in pairs.values()),key=lambda r:float(r['elapsed_s']))
        xs=[float(r['elapsed_s']) for r in data]
        ys=[number(r,'z_m') if r['status']=='LOCATED' else np.nan for r in data]
        axs[0,0].plot(xs,ys,'.-',label=f'{size}x{size}',lw=1,ms=3,color=colors[i])
    reference=config.get('reference_z_m')
    if reference is not None: axs[0,0].axhline(reference,color='black',ls='--',lw=1,label='Reference Z')
    axs[0,0].set(title='Accepted Z over time (gaps = rejected)',xlabel='Trial time (s)',ylabel='Optical Z (m)')
    if pairs: axs[0,0].legend()
    else: axs[0,0].text(.5,.5,'NO MATCHED DATA',ha='center',transform=axs[0,0].transAxes)
    labels=[f"{r['roi_size']}x{r['roi_size']}" for r in summary]
    specs=[(axs[0,1],'pass_rate','Acceptance / matched pairs','Pass rate (%)',100),
           (axs[1,0],'std_z_m','Accepted Z variation (stationary trials only)','Sample standard deviation (mm)',1000),
           (axs[1,1],'common_mae_m','MAE on pairs accepted by ALL sizes','Common-pair MAE (mm)',1000)]
    for ax,key,title,ylabel,scale in specs:
        for i,row in enumerate(summary):
            value=row[key]
            if value is not None:
                ax.bar(i,value*scale,color=colors[i]); ax.annotate(f'{value*scale:.2f}',(i,value*scale),ha='center',va='bottom',fontsize=10)
            else: ax.text(i,.04,'N/A',transform=ax.get_xaxis_transform(),ha='center')
        ax.set(xticks=range(len(labels)),xticklabels=labels,title=title,ylabel=ylabel)
        ax.margins(y=.2)
    axs[0,1].set_ylim(0,112)
    for ax in axs.flat: ax.grid(axis='y',alpha=.2); ax.set_axisbelow(True)
    fig.savefig(folder/'roi_comparison.png',dpi=160)
    fig.savefig(folder/'roi_comparison.pdf')
    plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('trial',type=Path)
    p.add_argument('--no-plots',action='store_true')
    a=p.parse_args()
    report=summarize(a.trial,not a.no_plots)
    print(json.dumps({k:report[k] for k in ('status','complete_pairs','incomplete_pairs_excluded','writer_count_matches')},indent=2))
    print('Saved report:',a.trial.resolve())
