# Usage:
#   python transform.py ./data/all_gt_poses_exo.csv ./data/all_gt_poses_exo1.csv
import csv, math, sys
from typing import List, Tuple

def quat_mul(q1,q2):
    w1,x1,y1,z1=q1; w2,x2,y2,z2=q2
    return (w1*w2-x1*x2-y1*y2-z1*z2,
            w1*x2+x1*w2+y1*z2-z1*y2,
            w1*y2-x1*z2+y1*w2+z1*x2,
            w1*z2+x1*y2-y1*x2+z1*w2)
def quat_conj(q): w,x,y,z=q; return (w,-x,-y,-z)
def quat_norm(q):
    w,x,y,z=q; n=math.sqrt(w*w+x*x+y*y+z*z) or 1.0
    return (w/n,x/n,y/n,z/n)
def quat_to_rot(q):
    w,x,y,z=q
    xx,yy,zz=x*x,y*y,z*z; xy,xz,yz=x*y,x*z,y*z; wx,wy,wz=w*x,w*y,w*z
    return [[1-2*(yy+zz), 2*(xy-wz),   2*(xz+wy)],
            [2*(xy+wz),   1-2*(xx+zz), 2*(yz-wx)],
            [2*(xz-wy),   2*(yz+wx),   1-2*(xx+yy)]]
def rot_vec(R,v):
    return [R[0][0]*v[0]+R[0][1]*v[1]+R[0][2]*v[2],
            R[1][0]*v[0]+R[1][1]*v[1]+R[1][2]*v[2],
            R[2][0]*v[0]+R[2][1]*v[1]+R[2][2]*v[2]]
def compose_pose(p_w_c1,q_w_c1,t12,q12):
    R=quat_to_rot(q_w_c1)
    t=rot_vec(R,t12)
    p=[p_w_c1[0]+t[0], p_w_c1[1]+t[1], p_w_c1[2]+t[2]]
    q=quat_norm(quat_mul(q_w_c1,q12))
    return p,q

def compute_alignment_transform()->Tuple[Tuple[float,float,float,float],List[float]]:
    # cam1 (Aria) world pose
    x1,y1,z1 = 0.7643741017815591, 1.7801150432995887, 0.04058623191220215
    qx1,qy1,qz1,qw1 = 0.8459594625636131, -0.21794367078394805, 0.030347919959384503, -0.485728471286623
    q_w_c1 = quat_norm((qw1,qx1,qy1,qz1))

    # solvePnP gives cam1->cam2 in cam2 frame (R21,t21)
    t21 = [0.04063719, -0.55304604, -0.478759]
    qw21,qx21,qy21,qz21 = 0.96796896, -0.22128914, -0.11190422, 0.039302
    q21 = quat_norm((qw21,qx21,qy21,qz21))

    # Invert to cam1_T_cam2 (R12,t12)
    q12 = quat_conj(q21)
    R12 = quat_to_rot(q12)
    t12 = [-v for v in rot_vec(R12, t21)]

    # A = cam2 in Aria world
    p_a, q_a = compose_pose([x1,y1,z1], q_w_c1, t12, q12)

    # B = given cam2 in Exo world (from CSV row)
    x2,y2,z2 = -1.478070204, 5.031958368, 0.062220084
    qx2,qy2,qz2,qw2 = -0.694345467, -0.030296305, 0.025378137, 0.718555813
    q_b = quat_norm((qw2,qx2,qy2,qz2))
    p_b = [x2,y2,z2]

    # Find X such that A ≈ X * B
    q_X = quat_norm(quat_mul(q_a, quat_conj(q_b)))
    R_X = quat_to_rot(q_X)
    RpB = rot_vec(R_X, p_b)
    t_X = [p_a[0]-RpB[0], p_a[1]-RpB[1], p_a[2]-RpB[2]]
    return q_X, t_X

def load_rows(path:str)->List[List[str]]:
    with open(path,'r',newline='') as f: return list(csv.reader(f,skipinitialspace=True))
def write_rows(path:str,rows:List[List[str]]):
    with open(path,'w',newline='') as f: csv.writer(f).writerows(rows)

def transform_csv(input_csv:str, output_csv:str):
    q_X, t_X = compute_alignment_transform()
    R_X = quat_to_rot(q_X)
    rows = load_rows(input_csv)
    if not rows: raise RuntimeError("Input CSV is empty")
    out = [rows[0]]
    for row in rows[1:]:
        if len(row) < 10: continue
        try:
            x,y,z = float(row[3]), float(row[4]), float(row[5])
            qx,qy,qz,qw = float(row[6]), float(row[7]), float(row[8]), float(row[9])
        except ValueError:
            continue
        p_prime = rot_vec(R_X,[x,y,z]); p_prime=[t_X[0]+p_prime[0], t_X[1]+p_prime[1], t_X[2]+p_prime[2]]
        q_prime = quat_norm(quat_mul(q_X,(qw,qx,qy,qz)))
        new = row[:]
        new[3]=f"{p_prime[0]:.12f}"; new[4]=f"{p_prime[1]:.12f}"; new[5]=f"{p_prime[2]:.12f}"
        new[6]=f"{q_prime[1]:.12f}"; new[7]=f"{q_prime[2]:.12f}"; new[8]=f"{q_prime[3]:.12f}"; new[9]=f"{q_prime[0]:.12f}"
        out.append(new)
    write_rows(output_csv,out)
    print(f"Wrote aligned CSV to {output_csv}")
    print(f"Alignment X: q_X(w,x,y,z)={q_X}, t_X={t_X}")

def main():
    if len(sys.argv)!=3:
        print("Usage: python apply_world_alignment.py input.csv output.csv"); sys.exit(1)
    transform_csv(sys.argv[1], sys.argv[2])

if __name__=="__main__":
    main()