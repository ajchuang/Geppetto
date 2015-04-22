.PHONY: all clean 

all:
	@javac -cp ./src:./lib/myo-java-0.8.2-SNAPSHOT.jar ./src/DataCollector.java
	@javac -cp ./src:./lib/myo-java-0.8.2-SNAPSHOT.jar ./src/MrGeppettoMyo.java
	@mv ./src/*.class ./obj/ 

clean:
	@rm ./obj/*.class
