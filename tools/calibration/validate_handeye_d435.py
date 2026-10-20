"""Offline new-session validation of a frozen fixed-camera transform; no refitting."""
import argparse
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
from object_hand.calibration import transform,mean_pose
from tools.calibration.calibrate_handeye_d435 import stats


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('calibration',type=Path)
    parser.add_argument('validation_session',type=Path)
    args=parser.parse_args()
    cal=json.loads(args.calibration.read_text());root=args.validation_session
    session=json.loads((root/'session.json').read_text())
    if session['camera_serial']!=cal['camera_serial']:raise ValueError('Camera serial mismatch')
    x=np.array(cal['T_ur_base_camera']);reference=np.array(cal['validation']['T_active_tcp_board_mean'])
    k=np.array(session['camera_matrix']);dist=np.array(session['distortion_coefficients'])
    if 'inverse_brown' in session['distortion_model'] and np.any(dist):raise ValueError('Inverse distortion requires explicit image correction')
    b=cal['board_model'];board=cv2.aruco.CharucoBoard((b['columns'],b['rows']),b['square_m'],b['marker_m'],cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    detector=cv2.aruco.CharucoDetector(board)
    output=root/'validation_frozen_calibration';output.mkdir(exist_ok=True)
    rows=[];fixed=[]
    for p in sorted(root.glob('*.png')):
        im=cv2.imread(str(p));cs,ids,_,mi=detector.detectBoard(im)
        marker_ids=[] if mi is None else mi.flatten().tolist()
        row={'image':p.name,'corners':0 if ids is None else len(ids)}
        if ids is None or len(ids)<8 or board.checkCharucoCornersCollinear(ids) or len(marker_ids)!=len(set(marker_ids)):
            row.update(accepted=False,reason='insufficient corners or duplicate board');rows.append(row);continue
        ob,ip=board.matchImagePoints(cs,ids);ok,r,t=cv2.solvePnP(ob,ip,k,dist)
        if not ok:
            row.update(accepted=False,reason='PnP failed');rows.append(row);continue
        projection,_=cv2.projectPoints(ob,r,t,k,dist)
        rms=float(np.sqrt(np.mean(np.sum((projection.reshape(-1,2)-ip.reshape(-1,2))**2,axis=1))))
        raw=json.loads(p.with_suffix('.json').read_text());before=raw['robot_before_image'];after=raw['robot_after_image']
        dq=float(np.max(np.abs(np.array(after['actual_q_rad'])-before['actual_q_rad'])))
        row.update(accepted=bool(rms<=1.5 and dq<=.001 and t[2,0]>0),reprojection_rms_px=rms)
        if row['accepted']:
            tcp=after['actual_tcp_pose_m_rotvec_rad']
            y=np.linalg.inv(transform(tcp[3:],tcp[:3]))@x@transform(r,t)
            fixed.append(y);row['T_active_tcp_board']=y.tolist()
        cv2.aruco.drawDetectedCornersCharuco(im,cs,ids);cv2.drawFrameAxes(im,k,dist,r,t,.05)
        cv2.imwrite(str(output/p.name),im);rows.append(row)
    if len(fixed)<3:raise ValueError('Fewer than three accepted validation samples')
    report={'refitted':False,'sample_count':len(rows),'accepted_samples':len(fixed),
            'calibration_sha256':hashlib.sha256(args.calibration.read_bytes()).hexdigest(),
            'compared_to_original_tcp_board':stats(fixed,reference),
            'within_new_session_consistency':stats(fixed,mean_pose(fixed)),'samples':rows,
            'assumptions':['Board mounting in gripper and TCP unchanged since original acquisition',
                           'Camera and robot base unchanged'],
            'limitation':'Board consistency only; absolute gripper/depth accuracy not validated'}
    (output/'report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='samples'},indent=2))
    print('Saved:',output/'report.json')


if __name__=='__main__':main()
