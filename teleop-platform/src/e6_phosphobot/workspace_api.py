"""Read-only, allowlisted assets. This API never connects to the robot."""
import json
from fastapi import HTTPException
from fastapi.responses import FileResponse
from .assembly_model import ASSETS, manifest


def install_workspace_api(router):
    @router.get('/e6/workspace')
    def workspace():
        try:
            info=manifest()
        except (OSError, ValueError):
            raise HTTPException(503, 'Workspace model unavailable')
        return dict(info, robot_url='/e6/workspace/assets/robot.urdf',
                    map_url='/e6/workspace/map', map_available=(ASSETS/'reachability.json').is_file())

    @router.get('/e6/workspace/map')
    def workspace_map():
        path=ASSETS/'reachability.json'
        if not path.is_file():raise HTTPException(404, 'Reachability sampling not available')
        value=json.loads(path.read_text(encoding='utf-8'))
        if value.get('model_id')!=manifest()['model_id']:
            raise HTTPException(409, 'Reachability map belongs to another model')
        return FileResponse(path,media_type='application/json')

    @router.get('/e6/workspace/assets/{name}')
    def workspace_asset(name:str):
        if name not in manifest()['files'] or '/' in name or '\\' in name:
            raise HTTPException(404, 'Unknown model asset')
        return FileResponse(ASSETS/name,media_type='application/xml' if name.endswith(('.urdf','.srdf')) else 'model/stl',
                            headers={'Cache-Control':'no-cache'})
