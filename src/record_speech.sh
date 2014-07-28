#!/bin/bash
arecord -vv -fdat $(rospack find gesture_rec)/test_data/recordings/rec$(date +"_%Y%m%d%H%M$S").wav