## Dump audio to wave

We provide tools to dump audio data from topic `/apollo/sensor/microphone` to wave files.

### How

To do so,
1. Change `WAV_SAVING_PATH` in `audiosaver.py` to the directory for saving wave files, which is default to `/tmp`.
2. Run command `python3 audiosaver.py` to start to listen to audio data from the topic `/apollo/sensor/microphone` (while playing a record, etc.).
3. Terminate the program with `Ctrl + C` when you want to stop listening & save data to wave files under the target directory.

By default, there are 6 audio channels, so 6 files will be generated -- one for each audio channel. For information of the audio channel, refer to [Microphone](../../drivers/microphone/README.md) and [Microphone Configuration](../../drivers/microphone/conf/respeaker.pb.txt).

### Other References

* Hardware Specification -- [Respeaker](../../../docs/11_Hardware%20Integration%20and%20Calibration/%E8%BD%A6%E8%BE%86%E9%9B%86%E6%88%90/%E4%BC%A0%E6%84%9F%E5%99%A8%E5%AE%89%E8%A3%85%20sensor%20installation/Microphone/Re_Speaker_USB_Mic_Array_Guide.md)
* Driver Configuration -- [Microphone](../../drivers/microphone/README.md)

