// New UI stateful frontend implementing the requested interaction flow
const recordBtn = document.getElementById('recordBtn');
const sendTextBtn = document.getElementById('sendTextBtn');
const manualText = document.getElementById('manualText');
const orb = document.getElementById('orb');
const orbInner = document.getElementById('orbInner');
const orbCaption = document.getElementById('orbCaption');
const spinner = document.getElementById('spinner');
const bars = document.getElementById('bars');
const messagesEl = document.getElementById('messages');
const replyAudio = document.getElementById('replyAudio');

let audioContext = null;
let micStream = null;
let analyser = null;
let rafId = null;
let mediaRecorder = null;
let recordedChunks = [];
let playbackSourceNode = null;

const State = {
  IDLE: 'idle', RECORDING: 'recording', PROCESSING: 'processing', SPEAKING: 'speaking', DONE: 'done'
};
let uiState = State.IDLE;

// create bars
function setupBars(count=8){
  bars.innerHTML = '';
  for(let i=0;i<count;i++){
    const b = document.createElement('div'); b.className='bar'; b.style.height='6px'; bars.appendChild(b);
  }
}
setupBars(8);

function setState(s){
  uiState = s;
  orb.className = 'orb '+(s===State.IDLE? 'idle': s===State.RECORDING? 'recording': s===State.SPEAKING? 'speaking':'');
  spinner.classList.toggle('active', s===State.PROCESSING);
  bars.classList.toggle('visible', s===State.RECORDING || s===State.SPEAKING);
  if(s===State.IDLE){ orbCaption.textContent = 'Tap to speak'; }
  if(s===State.RECORDING){ orbCaption.textContent = 'Listening...'; }
  if(s===State.PROCESSING){ orbCaption.textContent = 'Thinking...'; }
  if(s===State.SPEAKING){ orbCaption.textContent = 'Speaking...'; }
}

function appendMessage(kind, text, opts={}){
  const m = document.createElement('div'); m.className = 'message '+(kind==='user'?'user':'ai');
  const p = document.createElement('div'); p.className='text';
  p.textContent = text;
  m.appendChild(p);
  messagesEl.appendChild(m);
  messagesEl.scrollTop = messagesEl.scrollHeight;
  return p;
}

async function startRecording(){
  recordedChunks = [];
  try{
    const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
    micStream = stream;
    audioContext = new (window.AudioContext || window.webkitAudioContext)();
    const src = audioContext.createMediaStreamSource(stream);
    analyser = audioContext.createAnalyser(); analyser.fftSize = 256;
    src.connect(analyser);
    visualize();

    mediaRecorder = new MediaRecorder(stream);
    mediaRecorder.ondataavailable = e=>{ if(e.data.size>0) recordedChunks.push(e.data); };
    mediaRecorder.onstop = onRecordingStop;
    mediaRecorder.start();
    setState(State.RECORDING);
  }catch(e){ alert('microphone access denied or not available'); console.error(e); }
}

function stopRecording(){
  if(mediaRecorder && mediaRecorder.state !== 'inactive') mediaRecorder.stop();
  if(micStream){ micStream.getTracks().forEach(t=>t.stop()); micStream=null; }
  if(rafId){ cancelAnimationFrame(rafId); rafId=null; }
}

function visualize(){
  if(!analyser) return;
  const data = new Uint8Array(analyser.frequencyBinCount);
  const barsEls = Array.from(bars.children);
  function frame(){
    analyser.getByteFrequencyData(data);
    const step = Math.floor(data.length / barsEls.length);
    barsEls.forEach((el,i)=>{
      const slice = data.slice(i*step, (i+1)*step);
      const v = slice.reduce((a,b)=>a+b,0)/slice.length/255;
      el.style.height = Math.max(6, Math.round(v*120)) + 'px';
    });
    rafId = requestAnimationFrame(frame);
  }
  frame();
}

async function onRecordingStop(){
  setState(State.PROCESSING);
  // create blob and upload
  const blob = new Blob(recordedChunks, {type:'audio/webm'});
  const form = new FormData(); form.append('file', blob, 'rec.webm');

  // append user message
  const userTextEl = appendMessage('user', '(voice)');

  // transcribe then chat
  try{
    const r1 = await fetch('/api/transcribe', {method:'POST', body: form});
    const d1 = await r1.json();
    if(!r1.ok) throw new Error(d1.error||'transcribe failed');
    userTextEl.textContent = d1.text || '(no text)';

    // ai placeholder
    const aiTextEl = appendMessage('ai', '…');

    const r2 = await fetch('/api/chat', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({text:d1.text})});
    const d2 = await r2.json();
    if(!r2.ok) throw new Error(d2.error||'chat failed');
    // show reply as typewriter
    setState(State.SPEAKING);
    await playAndShowReply(d2.reply || '(no reply)', d2.audio, aiTextEl);
    setState(State.DONE);
    setTimeout(()=>setState(State.IDLE), 600);
  }catch(e){
    console.error(e); appendMessage('ai', 'Error: '+(e.message||e)); setState(State.IDLE);
  }
}

async function playAndShowReply(text, audioUrl, aiTextEl){
  // typewriter
  aiTextEl.textContent = '';
  function typewriter(el, str){
    return new Promise(resolve=>{
      let i=0; function step(){ if(i<=str.length){ el.textContent = str.slice(0,i); i++; setTimeout(step, 18); } else resolve(); } step();
    });
  }

  if(audioUrl){
    try{
      // load audio into hidden audio element
      replyAudio.src = audioUrl;
      // connect for visualizer (only create source node once per audio element)
      if(!audioContext) audioContext = new (window.AudioContext || window.webkitAudioContext)();
      if(!playbackSourceNode){
        playbackSourceNode = audioContext.createMediaElementSource(replyAudio);
        analyser = audioContext.createAnalyser(); analyser.fftSize = 256; 
        playbackSourceNode.connect(analyser); 
        analyser.connect(audioContext.destination);
      }
      visualize();
      // play and type simultaneously
      const playPromise = replyAudio.play();
      await Promise.all([ typewriter(aiTextEl, text), playPromise.catch(()=>{}) ]);
      // cleanup visualizer RAF
      if(rafId){ cancelAnimationFrame(rafId); rafId=null; }
    }catch(e){ console.error('play failed', e); await typewriter(aiTextEl, text); }
  } else {
    await typewriter(aiTextEl, text);
  }
}

recordBtn.addEventListener('click', ()=>{
  if(uiState===State.IDLE) startRecording(); else stopRecording();
});

sendTextBtn.addEventListener('click', async ()=>{
  const text = manualText.value.trim();
  if(!text) return;
  appendMessage('user', text);
  const aiEl = appendMessage('ai', '…');
  setState(State.PROCESSING);
  try{
    const r = await fetch('/api/chat', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({text})});
    const d = await r.json();
    if(!r.ok) throw new Error(d.error||'chat failed');
    setState(State.SPEAKING);
    await playAndShowReply(d.reply||'(no reply)', d.audio, aiEl);
    setState(State.DONE);
    setTimeout(()=>setState(State.IDLE), 600);
  }catch(e){ console.error(e); aiEl.textContent = 'Error: '+(e.message||e); setState(State.IDLE); }
  manualText.value = '';
});

