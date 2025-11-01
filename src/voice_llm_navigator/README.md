# voice_llm_navigator (MVP)

A small ROS2 package scaffold that implements a voice-to-LLM navigation bridge.

MVP features:
- Local/offline speech recognition via SpeechRecognition + PocketSphinx (if installed)
- Rule-based LocalLLMClient to map speech to navigation intents using currently-detected objects
- Publishes normalized commands to `/llm_goal` so the existing pipeline can act on them

Note about LLM requirement:
- The package can integrate with Ollama for local LLM parsing. If you want the
	Ollama-based behavior (no fallback), install Ollama and pull the Llama 3.1 8B
	model before launching the node. The node will raise an error if Ollama is not
	present when `LocalLLMClient` is initialized (user requested no fallback).

See the instructions below for installing Ollama and pulling the model.

How to run (after building with colcon):

```bash
# from workspace root
colcon build --packages-select voice_llm_navigator
. install/setup.bash
ros2 launch voice_llm_navigator voice_navigation.launch.py
```

For testing without a microphone, the node will prompt for text input on the terminal.
