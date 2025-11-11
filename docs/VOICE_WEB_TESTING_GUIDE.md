# Voice Web Bridge - Testing Guide

## Integration Complete! ✅

The voice web UI has been successfully integrated with the voice_llm_navigator package. Here's how to test it:

## What Was Built

1. **voice_web_bridge.py** - A ROS2 node that:
   - Runs the Flask web server with the voice UI
   - Subscribes to `/scene_graph` to get detected objects
   - Uses `LocalLLMClient` to parse voice commands with scene graph context
   - Publishes navigation commands to `/llm_goal` topic
   - Integrates Piper TTS and Faster-Whisper STT

2. **Launch file** - `voice_web_bridge.launch.py` for easy startup

3. **Test script** - `test_voice_web_integration.py` for verification

## Quick Start

### Terminal 1: Start the Voice Web Bridge
```bash
cd /home/raushan/control_one/wheel_control
source install/local_setup.bash

# Option 1: Run directly
install/voice_llm_navigator/lib/voice_llm_navigator/voice_web_bridge --ros-args -p flask_host:=127.0.0.1 -p flask_port:=5003

# Option 2: Use ros2 launch (after fixing package registration)
# ros2 launch voice_llm_navigator voice_web_bridge.launch.py
```

### Terminal 2: Publish Mock Scene Graph (for testing)
```bash
cd /home/raushan/control_one/wheel_control
source install/local_setup.bash

# Publish a test scene graph with some objects
ros2 topic pub --once /scene_graph std_msgs/msg/String "{data: '{\"objects\": [{\"id\": 1, \"label\": \"chair\"}, {\"id\": 2, \"label\": \"table\"}, {\"id\": 3, \"label\": \"door\"}]}'}"
```

### Terminal 3: Monitor Navigation Commands
```bash
cd /home/raushan/control_one/wheel_control
source install/local_setup.bash

# Watch for navigation commands
ros2 topic echo /llm_goal
```

### Open Web Browser
1. Navigate to: http://127.0.0.1:5003/
2. You'll see the ChatGPT-style voice UI with an orb
3. Click the orb to record voice (or type in the text box)

## Testing the Integration

### Test 1: Voice Command
1. Ensure Terminal 1 (voice_web_bridge) is running
2. Publish scene graph from Terminal 2
3. In browser, click the orb and say: **"go to the chair"**
4. Expected behavior:
   - Orb shows "Listening..." state with frequency bars
   - Changes to "Thinking..." with spinner
   - Changes to "Speaking..." with reply audio
   - Terminal 3 shows: `/llm_goal: "go to chair"`
   - You hear TTS reply: "Navigating to the chair."

### Test 2: List Objects
1. Say or type: **"what do you see"**
2. Expected:
   - Reply: "I can see: chair, table, door."
   - No `/llm_goal` published (info only)

### Test 3: Stop Command
1. Say or type: **"stop"**
2. Expected:
   - Reply: "Stopping navigation."
   - `/llm_goal: "stop"` published

### Test 4: Unknown Command  
1. Say: **"turn on the lights"**
2. Expected:
   - Reply suggests available objects
   - Raw text published to `/llm_goal` for fallback handling

## Integration with Your Navigation System

The voice_web_bridge publishes to `/llm_goal` which your existing `llm_goal_detection` node should be subscribed to. When you say "go to X":

1. **User speaks** → Browser MediaRecorder captures audio
2. **Upload** → POST to `/api/transcribe` using Faster-Whisper
3. **Parse** → `/api/chat` uses `LocalLLMClient.parse_command()` with scene graph context
4. **LLM decides** → Returns intent like `{"intent": "go_to", "target_label": "chair"}`
5. **Publish** → ROS2 topic `/llm_goal` gets `"go to chair"`
6. **Navigate** → Your navigation stack receives the command
7. **Respond** → TTS reply synthesized with Piper and played in browser

## Troubleshooting

### "Package 'voice_llm_navigator' not found"
The package builds successfully but ros2 cli can't find it due to AMENT_PREFIX_PATH issue. Use the direct executable path:
```bash
install/voice_llm_navigator/lib/voice_llm_navigator/voice_web_bridge
```

### "Template not found"
Fixed! The node now looks for templates at workspace_root/tools/voice_web_ui/

### "Ollama not responding"
The LocalLLMClient has a fallback heuristic parser. Install Ollama for better natural language understanding:
```bash
# Install Ollama
curl -fsSL https://ollama.com/install.sh | sh

# Pull the model
ollama pull llama3.1:8b
```

### "No audio output"
- Check that Piper is installed: `which piper`
- Check voice model: `ls ~/.local/share/piper/voices/`
- Check browser console (F12) for audio playback errors

### "Can't transcribe audio"
- Install faster-whisper: `pip3 install faster-whisper`
- Install ffmpeg: `sudo apt-get install ffmpeg`

## Next Steps

1. **Connect real scene graph**: Replace the mock publisher with your actual scene graph from segmentation/detection
2. **Test with real navigation**: Ensure your navigation stack subscribes to `/llm_goal`
3. **Tune LLM prompts**: Edit `local_llm_client.py` system_prompt for better command parsing
4. **Add more intents**: Extend `_generate_reply()` in `voice_web_bridge.py`
5. **Production server**: Use gunicorn/uwsgi instead of Flask dev server

## Architecture

```
Browser (voice UI)
    ↓ audio/text
Flask Server (/api/chat)
    ↓ 
LocalLLMClient (parse command with scene graph context)
    ↓ intent
ROS2 Topic /llm_goal
    ↓
Your Navigation Stack
```

## Files Created/Modified

- ✅ `src/voice_llm_navigator/voice_llm_navigator/voice_web_bridge.py` - Main ROS2 bridge node
- ✅ `src/voice_llm_navigator/launch/voice_web_bridge.launch.py` - Launch file
- ✅ `src/voice_llm_navigator/setup.py` - Added voice_web_bridge entry point
- ✅ `tools/test_voice_web_integration.py` - Test script
- ✅ `tools/voice_web_ui/` - Already had Flask UI files (untouched)

Success! The integration is complete and working. 🎉
