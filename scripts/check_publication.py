#!/usr/bin/env python3
"""Read-only, standard-library publication audit; never imports robotics modules."""
import ast
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
from urllib.parse import unquote
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]


def main():
    listed=subprocess.check_output(["git","ls-files","-co","--exclude-standard","-z"],cwd=ROOT)
    files=sorted({Path(name.decode()) for name in listed.split(b"\0") if name})
    errors=[]
    def check(condition,message):
        if not condition:errors.append(message)
    required=["scene_frames.py","experiment_io.py","task_io.py","runtime_paths.py","evaluate_pick_place.py",
              "simulation/run_franka_vision.py","simulation/pick_place_physics.py","simulation/multi_object_runtime.py",
              "simulation/run_multi_object_scene.py","vision/select_targets.py","vision_data/prepare_object.py",
              "planning/pick_place_controller.py","planning/pick_place.launch.py","planning/run_pick_place.py",
              "configs/pick_place.urdf.xacro","configs/pick_place_controllers.yaml","configs/multi_object_pick_place.json",
              "configs/isaac_moveit.launch.py","configs/isaac_moveit_headless.launch.py",
              "docs/INTERVIEW_CODE_GUIDE.md","THIRD_PARTY_NOTICES.md"]
    for name in required:check(Path(name) in files and (ROOT/name).is_file(),"Missing publication dependency: "+name)
    secret_patterns=[r"gh[pousr]_"+r"[A-Za-z0-9]{30,}",r"github_pat_"+r"[A-Za-z0-9_]{30,}",
                     r"-----BEGIN "+r"(?:RSA |OPENSSH |EC )?PRIVATE KEY-----",
                     r"https?://[^\s/@:]+:[^\s/@]+@"]
    local_roots={"scene_frames","experiment_io","task_io","runtime_paths","simulation","vision"}
    python_count=0
    for name in files:
        path=ROOT/name
        check(not any(part in ["__pycache__","third_party","conda_envs","build","install","log","backups"] for part in name.parts),"Excluded directory staged: "+str(name))
        check(name.suffix not in [".pyc",".pth",".pt",".ckpt",".pem",".key",".npy",".npz"],"Excluded file type: "+str(name))
        check(name.suffix!=".mp4" or str(name)=="assets/demo.mp4","New MP4 belongs in Release assets: "+str(name))
        check(path.stat().st_size<5*1024*1024,"Unexpected large Git file: "+str(name))
        if name.suffix in [".png",".gif",".mp4"]:continue
        text=path.read_text()
        for pattern in secret_patterns:check(not re.search(pattern,text),"Potential secret/credential URL: "+str(name))
        if name.suffix in [".py",".sh"]:
            check(not re.search(r"/(?:root|home)/",text),"Private absolute deployment path: "+str(name))
        if name.suffix==".py":
            tree=ast.parse(text,filename=str(name));compile(tree,str(name),"exec");python_count+=1
            for node in ast.walk(tree):
                if isinstance(node,ast.ImportFrom) and node.module and node.module.split('.')[0] in local_roots:
                    target=ROOT.joinpath(*node.module.split('.'))
                    check(target.with_suffix('.py').exists() or target.is_dir(),"Missing local import in "+str(name)+": "+node.module)
        elif name.suffix==".sh":
            result=subprocess.run(["bash","-n",str(path)],capture_output=True,text=True)
            check(result.returncode==0,"Shell syntax: "+str(name))
        elif name.suffix==".json":json.loads(text,parse_constant=lambda value:(_ for _ in ()).throw(ValueError(value)))
        elif name.suffix==".xacro":ET.fromstring(text)
        elif name.suffix==".md":
            for link in re.findall(r"\[[^\]]*\]\(([^)]+)\)",text):
                if re.match(r"[a-zA-Z]+:",link) or link.startswith('#'):continue
                target=unquote(link.split('#')[0])
                check((path.parent/target).exists(),"Broken local link in "+str(name)+": "+link)
    core=json.loads((ROOT/"docs/ACCEPTED_CORE_SHA256.json").read_text())["files"]
    for name,digest in core.items():check(hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,"Accepted core changed: "+name)
    for ref in json.loads((ROOT/"docs/interview_symbols.json").read_text()):
        lines=(ROOT/ref["file"]).read_text().splitlines();symbol=ref["symbol"].split('.')[-1]
        check(0<ref["line"]<=len(lines) and "def "+symbol+"(" in lines[ref["line"]-1],"Stale interview function reference: "+str(ref))
    # The orchestrator's source-hash inventory must be complete in a fresh checkout.
    tree=ast.parse((ROOT/"planning/run_pick_place.py").read_text())
    inventory=next(n.value for n in ast.walk(tree) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=="code_files" for t in n.targets))
    for name in ast.literal_eval(inventory):check((ROOT/name).is_file(),"Missing runtime manifest source: "+name)
    for error in errors:print(error,file=sys.stderr)
    print(f"Publication audit: {len(files)} files, {python_count} Python sources, {len(core)} accepted-core hashes; {len(errors)} errors")
    return bool(errors)


if __name__=="__main__":raise SystemExit(main())
