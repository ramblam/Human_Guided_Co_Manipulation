# Speech recognition and TTS
We used [Creoir EdgeVUI SDK](https://creoir.com/edgevui/), which provides tools for automatic speech recognition and text-to-speech, for human-robot and robot-human communication. If using that, one can run:

**1. terminal:**
```
export LD_LIBRARY_PATH=/usr/lib/cerence
EdgeVUI --language="enu" --verbose=1
```

**2. terminal:**
```
cd voice_tts_ws
source install/setup.bash
cd
cd /usr/share/creoir/python
python3 collins_compliance_and_tracking_new.py
```

Or one can replace that with other speech recognition tools and use the same action interface.
