"""Read-only SSH particle/map display. No flight commands or services."""
import math
from pathlib import Path
import shutil
import sys
import time

import numpy as np


class AsciiMap:
    def __init__(self, width=72, height=30, bounds=(-3.15,3.15,-3.15,3.15)):
        self.width, self.height, self.bounds = width, height, bounds
        self.walls = np.full((height,width),' ',dtype='<U1')

    def cell(self, x, y):
        xmin,xmax,ymin,ymax = self.bounds
        if not np.isfinite([x,y]).all() or not xmin<=x<=xmax or not ymin<=y<=ymax:
            return None
        return (min(self.height-1,int((ymax-y)/(ymax-ymin)*(self.height-1))),
                min(self.width-1,int((x-xmin)/(xmax-xmin)*(self.width-1))))

    def add_wall(self, start, end):
        a,b = self.cell(*start[:2]),self.cell(*end[:2])
        if a is None or b is None:
            return
        dr,dc = b[0]-a[0], b[1]-a[1]
        symbol = '-' if abs(dc)>2*abs(dr) else ('|' if abs(dr)>2*abs(dc) else ('\\' if dr*dc>0 else '/'))
        for t in np.linspace(0,1,max(abs(dr),abs(dc))+1):
            self.walls[round(a[0]+t*dr),round(a[1]+t*dc)] = symbol

    def lines(self, particles, pose):
        grid = self.walls.copy()
        outside = 0
        for p in particles:
            cell = self.cell(p[0],p[1])
            if cell is None:outside+=1
            else:grid[cell]='.'
        if pose is not None:
            cell = self.cell(pose[0],pose[1])
            if cell is not None:grid[cell]='#'
        return ['+'+'-'*self.width+'+']+['|'+''.join(row)+'|' for row in grid]+['+'+'-'*self.width+'+'],outside


def load_walls(directory, canvas, height=.8):
    import trimesh
    for path in sorted(Path(directory).glob('*.obj')):
        if 'floor' in path.name:
            continue
        mesh = trimesh.load(path,force='mesh',process=False)
        segments = trimesh.intersections.mesh_plane(mesh,[0,0,1],[0,0,height])
        for start,end in segments:
            canvas.add_wall(start,end)


class LiveLogView:
    """Small panel in existing launch output; no terminal control or extra process."""
    def __init__(self, directory):
        self.canvas=AsciiMap(61,23)
        load_walls(directory,self.canvas)
        self.pose=None;self.particles=[];self.at=-float('inf');self.source_stamp=None

    def update(self, pose, particles, source_stamp):
        self.pose=np.asarray(pose).copy()
        self.particles=np.asarray(particles).copy()
        self.at=time.monotonic();self.source_stamp=source_stamp

    def text(self, ros_now, error=''):
        age=time.monotonic()-self.at
        source_age=float('inf') if self.source_stamp is None else ros_now-self.source_stamp
        live=age<1 and -.05<=source_age<1 and not error
        picture,outside=self.canvas.lines(self.particles,self.pose)
        where='Waiting for localization' if self.pose is None else (
            f'x={self.pose[0]:+.2f} y={self.pose[1]:+.2f} z={self.pose[2]:.2f} m '
            f'yaw={math.degrees(self.pose[3]):+.0f} deg')
        return '\n'.join(['', 'WALK LOCALIZATION: '+('LIVE' if live else 'STALE / WAITING'),where,
            f'# estimate | . particles ({len(self.particles)}, {outside} outside) | walls: - | / \\',
            'Map +Y up / +X right; nominal walls at z=0.80 m',*picture,
            f'Frame age {age:.1f}s | '+(error[:120] if error else 'Display is read-only; spread is not proof of accuracy')])


class Terminal:
    """ANSI alternate screen; transmit only changed rows, no scrolling frames."""
    def __init__(self, stream):
        self.stream,self.previous = stream,[]
        stream.write('\033[?1049h\033[?25l\033[2J');stream.flush()

    def draw(self, lines):
        output=[]
        for i,line in enumerate(lines):
            if i>=len(self.previous) or line!=self.previous[i]:
                output.append(f'\033[{i+1};1H{line}\033[K')
        self.stream.write(''.join(output));self.stream.flush()
        self.previous=lines

    def close(self):
        self.stream.write('\033[?25h\033[?1049l');self.stream.flush()


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseArray
    from nav_msgs.msg import Odometry
    from ament_index_python.packages import get_package_share_directory
    if not sys.stdout.isatty():
        raise SystemExit('Use an interactive SSH terminal (ssh -t); do not pipe this display.')
    rclpy.init(args=args)
    node=Node('course_ascii_view')
    rate=node.declare_parameter('refresh_hz',5.).value
    slice_z=node.declare_parameter('wall_slice_z_m',.8).value
    if not 0<rate<=10 or not math.isfinite(slice_z):
        raise ValueError('refresh_hz must be (0,10]; wall_slice_z_m must be finite')
    columns,rows=shutil.get_terminal_size((80,40))
    if columns<44 or rows<20:
        raise SystemExit('Terminal too small; resize to at least 44 columns x 20 rows.')
    canvas=AsciiMap(min(96,columns-3),min(36,rows-9))
    load_walls(Path(get_package_share_directory('course_localization'))/'map',canvas,slice_z)
    state=dict(particles=[],pose=None,particle_at=-float('inf'),pose_at=-float('inf'),
               pose_stamp=None,particle_stamp=None,frame_error='')
    def on_particles(msg):
        if msg.header.frame_id!='course_map':
            state['frame_error']='Particle frame mismatch';return
        state.update(particles=[(p.position.x,p.position.y) for p in msg.poses],particle_at=time.monotonic(),
                     particle_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
    def on_pose(msg):
        if msg.header.frame_id!='course_map':
            state['frame_error']='Pose frame mismatch';return
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        state.update(pose=(p.x,p.y,p.z,yaw),pose_at=time.monotonic(),
                     pose_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
    node.create_subscription(PoseArray,'/course/particles',on_particles,1)
    node.create_subscription(Odometry,'/course/particle_pose',on_pose,1)
    terminal=Terminal(sys.stdout)
    def draw():
        now=time.monotonic();ros_now=node.get_clock().now().nanoseconds*1e-9
        age=max(now-state['pose_at'],now-state['particle_at'])
        source_age=max([ros_now-s for s in (state['pose_stamp'],state['particle_stamp']) if s is not None],default=float('inf'))
        label='LIVE' if age<1 and -.05<=source_age<1 and not state['frame_error'] else 'STALE / WAITING'
        picture,outside=canvas.lines(state['particles'],state['pose'])
        pose=state['pose'];where='No estimated position received' if pose is None else f'x={pose[0]:+.2f} y={pose[1]:+.2f} z={pose[2]:.2f} m yaw={math.degrees(pose[3]):+.0f} deg'
        width=canvas.width+2
        lines=[f'WALK TEST - READ ONLY - {label}', '# estimated center   . particles   | - / \\ nominal walls',
               where,f'Particles: {len(state["particles"])}  outside view: {outside}  age: {age:.1f}s',
               f'Map +Y up, +X right; wall slice z={slice_z:.2f} m']+picture+[
               state['frame_error'] or 'Particle spread is uncertainty, not proof of accuracy. Ctrl-C exits.']
        terminal.draw([line[:width] for line in lines])
    node.create_timer(1/rate,draw)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        terminal.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
