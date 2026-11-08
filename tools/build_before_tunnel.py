from pathlib import Path
import sys,json,hashlib
import numpy as np
from scipy.interpolate import CubicSpline
from shapely.geometry import Polygon,LineString,Point
import argparse
parser=argparse.ArgumentParser()
parser.add_argument('--scene-root',type=Path,required=True,help='Simulator assets folder containing scene_real.xml and scripts/check_variant_clearance.py')
parser.add_argument('--error-margin',type=float,default=0.,help='Extra horizontal/vertical error envelope for stress check')
args=parser.parse_args()
ROOT=args.scene_root.resolve();sys.path.insert(0,str(ROOT/'scripts'))
from check_variant_clearance import meshes,clip
repo=Path(__file__).resolve().parents[1]
out=repo/'docs/before_tunnel'
out.mkdir(exist_ok=True)
src=repo/'src/course_localization/trajectory/course.npz'
f=np.load(src);t,p,y=f['time'],f['position'],np.unwrap(f['yaw'])
p=p.copy()
bump=np.maximum(0.,1.-((t-9.7)/1.2)**2)**3
p[:,0]-=.07*bump
j=int(np.argmin(abs(t-16.0)));T=2.5;end=np.array([2.0,1.70,p[0,2]])
sp=CubicSpline(t,p);sy=CubicSpline(t,y)
def segment(a,v,acc,b):
 c=np.zeros((6,)+np.shape(a));c[:3]=[a,v,acc/2]
 c[3:]=np.linalg.solve([[T**3,T**4,T**5],[3*T*T,4*T**3,5*T**4],[6*T,12*T*T,20*T**3]],
  [b-a-v*T-acc*T*T/2,-v-acc*T,-acc])
 u=np.linspace(0,T,1201)
 return u,sum(u.reshape((-1,)+(1,)*np.ndim(a))**k*c[k] for k in range(6))
u,q=segment(p[j],sp(t[j],1),sp(t[j],2),end)
_,qy=segment(y[j],sy(t[j],1),sy(t[j],2),y[j])
nt=np.r_[t[:j],t[j]+u,t[j]+T+1];np_=np.vstack([p[:j],q,end]);ny=np.r_[y[:j],qy,y[j]]
# Re-time to 0.25 m/s reference; command clamp is 0.3048 m/s, retaining feedback headroom.
peak=np.max(np.linalg.norm(np.diff(np_,axis=0),axis=1)/np.diff(nt));nt=nt[0]+(nt-nt[0])*max(1,peak/.25)
np.savez_compressed(repo/'src/course_localization/trajectory/before_tunnel.npz',time=nt,position=np_,yaw=ny)
objects=meshes(ROOT/'scene_real.xml');contacts=[];closest=(float('inf'),'')
# sample each 1 cm of path, inflated by 5 mm to cover midpoint displacement
arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(np_,axis=0),axis=1))]
samples=np.linspace(0,arc[-1],int(np.ceil(arc[-1]/.01))+1)
poses=np.column_stack([np.interp(samples,arc,np_[:,i]) for i in range(3)])
R=.335+args.error_margin
for name,mesh in objects:
 tri=mesh.triangles;lo=tri.min(1);hi=tri.max(1)
 for i,pos in enumerate(poses):
  lower=pos-[R,R,.121+args.error_margin];upper=pos+[R,R,.075+args.error_margin]
  if np.any(mesh.bounds[0]>upper) or np.any(mesh.bounds[1]<lower):continue
  for k in np.flatnonzero(np.all(lo<=upper,axis=1)&np.all(hi>=lower,axis=1)):
   poly=clip(clip(list(tri[k]),lower[2],True),upper[2],False)
   if len(poly)<2:continue
   xy=np.array(poly)[:,:2];sh=Polygon(xy) if len(poly)>=3 else LineString(xy);d=sh.distance(Point(pos[:2]))
   if d-R<closest[0]:closest=(d-R,name)
   if d<R:contacts.append(dict(object=name,arc_m=float(samples[i]),point=pos.tolist(),overlap_m=float(R-d)));break
report=dict(source_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),endpoint=end.tolist(),join_original_time_s=float(t[j]),
 duration_s=float(nt[-1]),distance_m=float(arc[-1]),max_reference_speed_mps=float(np.max(np.linalg.norm(np.diff(np_,axis=0),axis=1)/np.diff(nt))),
 tunnel_front_y=1.219,endpoint_front_clearance_m=float(end[1]-1.219-R),
 sampled_envelope_radius_m=R,additional_xyz_error_margin_m=args.error_margin,trajectory_sha256=hashlib.sha256((repo/'src/course_localization/trajectory/before_tunnel.npz').read_bytes()).hexdigest(),overlap_count=len(contacts),contacts=contacts[:20],closest_tested_triangle=closest,
 scope='Photo-reconstructed mesh and box geometry, level drone radius .32 plus .01 margin plus .005 sampling allowance and additional_xyz_error_margin_m. Floor and cage net omitted. Not flight validation or proof of bounded tracking/localization error.')
(out/('clearance_stress_report.json' if args.error_margin else 'geometry_report.json')).write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))

if not args.error_margin:
 assert not contacts, 'Reference has nominal envelope overlaps; inspect report before using'
