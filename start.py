from naoqi import ALProxy

tts = ALProxy("ALTextToSpeech", "10.1.5.26", 9559)
tts.say("Hello, iam connected")