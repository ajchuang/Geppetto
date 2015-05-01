#!/usr/bin/python
import speech_recognition as sr

r = sr.Recognizer()

while True:
    with sr.Microphone () as source:                 # use the default microphone as the audio source
   
        try:
            print 'Please say the command: '
            audio = r.listen (source)                   # listen for the first phrase and extract it into audio data
            cmd = r.recognize (audio)


            if cmd == 'go':
                print 'go forward'

            elif cmd == 'stop':
                print 'stop now'

            elif cmd == 'turn left':
                print 'turn left'

            elif cmd == 'turn right':
                print 'turn right'

            elif cmd == 'exit':
                print 'Bye-bye'
                break
            else:
                print 'cmd: ' + cmd
 
        except LookupError:                         # speech is unintelligible
            print "[ERROR] Could not understand audio"
            pass
