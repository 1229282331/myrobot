"""Vendor the complete pinned URDFViewer with bounded lifecycle/error fixes."""
from pathlib import Path
import hashlib,json,difflib
ROOT=Path(__file__).resolve().parents[1]
package=ROOT/'upstream/phosphobot/dashboard/node_modules/urdf-loader'
assert json.loads((package/'package.json').read_text())['version']=='0.13.1'
source=(package/'src/urdf-viewer-element.js').read_text()
value=source
def replace(old,new):
    global value
    assert old in value,old
    value=value.replace(old,new)
replace("import URDFLoader from './URDFLoader.js';","import URDFLoader from 'urdf-loader';")
replace('loader.loadMeshCb = this.loadMeshFunc;', 'if (this.loadMeshFunc) loader.loadMeshCb = this.loadMeshFunc;')
replace('THREE.PCFSoftShadowMap', 'THREE.PCFShadowMap')
replace('this._requestId = 0;','this._requestId = 0;\n        this._disposed = false;\n        this._materials = new Set();')
replace('cancelAnimationFrame(this._renderLoopId);','''cancelAnimationFrame(this._renderLoopId);
        cancelAnimationFrame(this._loadFrame);
        this._disposed = true;
        this._requestId++;
        this.controls.dispose();
        this._disposeObject(this.scene);
        this._materials.forEach(m => m.dispose());
        this._materials.clear();
        this._collisionMaterial.dispose();
        this.renderer.dispose();
        this.renderer.forceContextLoss();''')
replace('    attributeChangedCallback(attr, oldval, newval) {','''    _disposeObject(object) {
        if (!object) return;
        object.traverse(node => {
            node.geometry?.dispose();
            if (node.material) {
                const materials = Array.isArray(node.material) ? node.material : [node.material];
                materials.forEach(m => { m.map?.dispose(); m.dispose(); });
            }
        });
    }

    attributeChangedCallback(attr, oldval, newval) {''')
replace('        const currSize = r.getSize(tempVec2);','        if (!w || !h || this._disposed) return;\n        const currSize = r.getSize(tempVec2);')
replace('r.setPixelRatio(window.devicePixelRatio);','r.setPixelRatio(Math.min(window.devicePixelRatio, 2));')
replace('this.robot.traverse(c => c.dispose && c.dispose());','this._disposeObject(this.robot);')
replace('        requestAnimationFrame(() => {\n\n            this._loadUrdf', '        this._loadFrame = requestAnimationFrame(() => {\n\n            if (this._disposed) return;\n            this._loadUrdf')
replace('            const manager = new THREE.LoadingManager();','''            const manager = new THREE.LoadingManager();
            let failed = false;
            const reportError = error => {
                failed = true;
                if (!this._disposed) this.dispatchEvent(new CustomEvent('urdf-error', {detail: String(error)}));
            };
            manager.onError = reportError;''')
replace('                if (this._requestId !== requestId) {\n\n                    robot.traverse(c => c.dispose && c.dispose());','                if (this._disposed || failed || !robot || this._requestId !== requestId) {\n\n                    this._disposeObject(robot);')
replace('                this.robot = robot;','''                robot.traverse(node => {
                    if (node.material) (Array.isArray(node.material) ? node.material : [node.material]).forEach(m => this._materials.add(m));
                });
                this.robot = robot;''')
replace('loader.load(urdf, model => robot = model);','loader.load(urdf, model => robot = model, undefined, reportError);')
target=ROOT/'frontend/vendor/urdf-viewer-element.js';target.write_text(value,encoding='utf8',newline='\n')
assert (target.parent/'urdf-viewer-LICENSE').is_file(), 'Pinned Apache-2.0 license must be present'
patch=ROOT/'patches/viewer';patch.mkdir(exist_ok=True)
(patch/'0001-lifecycle.patch').write_text(''.join(difflib.unified_diff(source.splitlines(True),value.splitlines(True),fromfile='upstream/src/urdf-viewer-element.js',tofile='frontend/vendor/urdf-viewer-element.js')),encoding='utf8',newline='\n')
(patch/'source.json').write_text(json.dumps(dict(package='urdf-loader',version='0.13.1',commit='24feef2c267496d22d17bbbb5c1bf5b86dcf174f',license='Apache-2.0',upstream='https://github.com/gkjohnson/urdf-loaders',path='javascript/src/urdf-viewer-element.js',upstream_sha256=hashlib.sha256(source.encode()).hexdigest(),local_sha256=hashlib.sha256(value.encode()).hexdigest()),indent=2))
print('Complete URDFViewer vendored; lifecycle patch recorded')
