#!/usr/bin/env python3
"""Minimal local preview server for Mac (no rclpy needed)."""
import os, re, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'orangepi/hmi'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'shared'))

from flask import Flask, render_template, jsonify, request

BAG_DIR = os.path.join(os.path.dirname(__file__), 'bags')

app = Flask(__name__, template_folder='orangepi/hmi/templates')

_BAG_RE = re.compile(r'^[a-zA-Z0-9_\-]+$')

def _valid(name):
    return bool(name and _BAG_RE.match(name))


@app.route('/preview/<name>')
def preview_page(name):
    if not _valid(name):
        return 'Invalid bag name', 400
    if not os.path.isdir(os.path.join(BAG_DIR, name)):
        return 'Bag not found', 404
    return render_template('preview.html', bag_name=name)


@app.route('/api/bags/<name>/preview')
def api_bag_preview(name):
    if not _valid(name):
        return jsonify({'ok': False, 'msg': 'Invalid name'}), 400
    bag_path = os.path.join(BAG_DIR, name)
    if not os.path.isdir(bag_path):
        return jsonify({'ok': False, 'msg': 'Bag not found'}), 404

    max_clouds       = max(1,   min(request.args.get('max_clouds',       25,    type=int),   300))
    max_points       = max(1000,min(request.args.get('max_points',       150000,type=int),   1000000))
    target_rotations = max(0.5, min(request.args.get('target_rotations', 2.5,   type=float), 30.0))
    mount_pitch      = max(-30.,min(request.args.get('mount_pitch',      0.0,   type=float), 30.0))
    mount_yaw        = max(-30.,min(request.args.get('mount_yaw',        0.0,   type=float), 30.0))
    angle_offset     = max(-180.,min(request.args.get('angle_offset',    0.0,   type=float), 180.0))
    center_z_raw = request.args.get('center_z', None, type=float)
    center_z = max(-0.5, min(center_z_raw, 0.5)) if center_z_raw is not None else None

    try:
        from bag_preview import build_bag_preview
        result = build_bag_preview(bag_path, max_clouds=max_clouds, max_points=max_points,
                                   target_rotations=target_rotations,
                                   mount_pitch_deg=mount_pitch, mount_yaw_deg=mount_yaw,
                                   angle_offset_deg=angle_offset, center_z=center_z)
    except (FileNotFoundError, ValueError) as e:
        return jsonify({'ok': False, 'msg': str(e)}), 400
    except Exception as e:
        return jsonify({'ok': False, 'msg': f'Preview failed: {e}'}), 500

    resp = app.response_class(result['buf'], mimetype='application/octet-stream')
    resp.headers['X-Point-Count']         = str(result['points'])
    resp.headers['X-Cloud-Count']         = str(result['clouds'])
    resp.headers['X-Cloud-Total']         = str(result['clouds_total'])
    resp.headers['X-Low-Coverage-Clouds'] = str(result['low_coverage_clouds'])
    resp.headers['X-Encoder-Source']      = result['encoder_source']
    resp.headers['X-Rotations-Captured']  = str(result['rotations_captured'])
    resp.headers['X-Compute-S']           = str(result['compute_s'])
    resp.headers['X-Gravity-Source']      = result.get('gravity_source', 'none')
    return resp


if __name__ == '__main__':
    port = int(os.environ.get('PORT', 5001))
    print(f'Preview server → http://localhost:{port}/preview/<bag_name>')
    app.run(host='0.0.0.0', port=port, debug=False)
