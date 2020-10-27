import 'package:audio_service/audio_service.dart';
import 'package:flutter/cupertino.dart';


CupertinoButton playButton = CupertinoButton(
  child: Icon(
    CupertinoIcons.play,
    size: 50,
  ),
  onPressed: AudioService.play,
);

CupertinoButton pauseButton = CupertinoButton(
  child: Icon(
    CupertinoIcons.pause,
    size: 50,
  ),
  onPressed: AudioService.pause,
);

CupertinoButton previousButton(bool enabled) => CupertinoButton(
  child: Icon(
    CupertinoIcons.backward_end,
    size: 50,
  ),
  onPressed: enabled ? AudioService.skipToPrevious : null,
);

CupertinoButton nextButton(bool enabled) => CupertinoButton(
  child: Icon(
    CupertinoIcons.forward_end,
    size: 50,
  ),
  onPressed: enabled ? AudioService.skipToNext : null,
);