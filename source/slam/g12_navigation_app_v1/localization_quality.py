"""Shared usable/quality distinction for local and map registration."""
import copy,math

def classify(overlap,median,condition,converged=True):
    if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in (overlap,median,condition)):return 'unusable'
    if overlap<.35 or median>.35 or condition<1e-8:return 'unusable'
    return 'tracking' if converged and overlap>=.65 and median<=.16 and condition>=1e-6 else 'degraded'

def navigation_config(original):
    cfg=copy.deepcopy(original)
    cfg['registration'].update(overlap_min=.35,median_nn_max_m=.35,plane_rmse_max_m=.30,
        min_plane_points=80,condition_min=1e-8,source_rank_min=1e-5)
    cfg['relocalization']['confirmations']=1
    return cfg
