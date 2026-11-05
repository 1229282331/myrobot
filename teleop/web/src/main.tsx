import React from "react";
import { createRoot } from "react-dom/client";
import App from "./App";
import HardwareApp from "./HardwareApp";
import "./style.css";
fetch('/api/state').then(r=>r.json()).then(state=>{
  createRoot(document.getElementById('root')!).render(<React.StrictMode>{state.backend.startsWith('hardware-')?<HardwareApp controlBackend={state.backend==='hardware-control'}/>:<App/>}</React.StrictMode>);
}).catch(()=>{document.getElementById('root')!.textContent='PC服务暂不可用，请检查启动终端并刷新。'});
