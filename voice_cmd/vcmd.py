#!/usr/bin/python
import speech_recognition as sr

r = sr.Recognizer()

while True:
    with sr.Microphone() as source:                 # use the default microphone as the audio source
   
        print 'Please say the command: '
        audio = r.listen (source)                   # listen for the first phrase and extract it into audio data
        cmd = r.recognize (audio)

        try:
            print "Command: " + cmd                 # recognize speech using Google Speech Recognition
            
            if cmd == 'exit':
                break
 
        except LookupError:                         # speech is unintelligible
            print "[ERROR] Could not understand audio"

