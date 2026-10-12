#pragma once
#include <Arduino.h>
// Served by the vehicle ESP32. HTTP runs over TCP, using the ESP32 Wi-Fi AP.
const char CONTROL_PAGE[] PROGMEM = R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Dual Mode Robot</title>
<style>
*{box-sizing:border-box}body{font-family:system-ui,sans-serif;max-width:430px;margin:32px auto;padding:0 20px;background:#f4f6f8;color:#182739}
h1{font-size:26px}button{font:inherit;border:1px solid #aab8c8;border-radius:12px;padding:18px;background:#fff;color:inherit;touch-action:none;user-select:none}
button:active{background:#d8e9fa}button:disabled{opacity:.4}.pad{display:grid;grid-template-columns:repeat(3,1fr);gap:12px;margin-top:24px}
.forward{grid-column:2}.left{grid-column:1}.right{grid-column:3}.reverse{grid-column:2}.stop{width:100%;margin-top:22px;background:#9c2833;color:white;border:0;font-weight:700}
#status{padding:12px;background:white;border-radius:10px;min-height:70px}small{display:block;margin-top:16px;line-height:1.5}
</style></head><body>
<h1>Dual Mode Robot</h1>
<p>Select Gesture or Wi-Fi control using the physical button on the robot.</p>
<div id="status" role="status" aria-live="polite">Connecting to the vehicle...</div>
<div class="pad">
<button class="forward" data-command="F" disabled>Forward</button>
<button class="left" data-command="L" disabled>Left</button>
<button class="right" data-command="R" disabled>Right</button>
<button class="reverse" data-command="B" disabled>Reverse</button>
</div>
<button class="stop" id="stop">STOP</button>
<small>In Wi-Fi mode, hold a direction button to move and release it to stop.
STOP locks movement in either mode. Press the robot's physical mode button to resume.</small>
<script>
const buttons=[...document.querySelectorAll('[data-command]')];
const status=document.getElementById('status');
let state=null, held='S', pointer=null, busy=false, pendingStop=false;
let needsCommand=false, lastPoll=0;
const motionNames={F:'Forward',B:'Reverse',L:'Left',R:'Right',S:'Stopped'};
function releaseHold(){held='S';pointer=null;needsCommand=true;}
function showState(next){
  if(!state || next.mode_version!==state.mode_version ||
     next.mode!=='web' || next.halted) releaseHold();
  state=next;
  const enabled=state.mode==='web'&&!state.halted;
  buttons.forEach(button=>button.disabled=!enabled);
  const modeName=state.mode==='web'?'Wi-Fi control':'Gesture control';
  const extra=state.halted?'Press the physical mode button to resume.':
              state.needs_neutral?'Level the glove before moving.':'';
  status.textContent=modeName+' | '+(motionNames[state.command]||state.command)+'. '+extra;
}
async function networkTick(){
  if(busy) return;
  let url='/state', options={cache:'no-store'};
  if(pendingStop){
    pendingStop=false;
    url='/stop';options.method='POST';
  }else if(state && state.mode==='web' && !state.halted &&
           !document.hidden && (held!=='S'||needsCommand)){
    const command=held;needsCommand=false;
    url='/command';
    options.method='POST';
    options.headers={'Content-Type':'application/x-www-form-urlencoded'};
    options.body=new URLSearchParams({value:command,mode_version:String(state.mode_version)});
  }else if(Date.now()-lastPoll<250) return;
  busy=true;lastPoll=Date.now();
  const aborter=new AbortController();
  const timer=setTimeout(()=>aborter.abort(),800);
  try{
    const response=await fetch(url,{...options,signal:aborter.signal});
    if(!response.ok) throw new Error('Command rejected. Check the physical mode selection.');
    showState(await response.json());
  }catch(error){
    releaseHold();state=null;
    buttons.forEach(button=>button.disabled=true);
    status.textContent='Checking connection and mode. '+error.message;
  }finally{clearTimeout(timer);busy=false;}
}
buttons.forEach(button=>{
  button.addEventListener('pointerdown',event=>{
    if(!state || state.mode!=='web' || state.halted || pointer!==null) return;
    event.preventDefault();pointer=event.pointerId;held=button.dataset.command;
    needsCommand=true;button.setPointerCapture(event.pointerId);networkTick();
  });
  ['pointerup','pointercancel','lostpointercapture'].forEach(type=>{
    button.addEventListener(type,event=>{
      if(event.pointerId===pointer){releaseHold();networkTick();}
    });
  });
});
document.getElementById('stop').addEventListener('click',()=>{
  releaseHold();pendingStop=true;networkTick();
});
function stopOnLeave(){
  releaseHold();
  if(state && state.mode==='web' && !state.halted)
    navigator.sendBeacon('/command',new URLSearchParams({
      value:'S',mode_version:String(state.mode_version)
    }));
}
window.addEventListener('blur',()=>{releaseHold();networkTick();});
window.addEventListener('pagehide',stopOnLeave);
document.addEventListener('visibilitychange',()=>{if(document.hidden)stopOnLeave();});
setInterval(networkTick,100);networkTick();
</script></body></html>)HTML";

