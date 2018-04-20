# GrammarTester

the testGrammar.py is a initial draft of a Grammar tester for the eegpsr task in the Robocup@Home.
It takes a file with Commands that are supposed to be checkt. Example: EEGPSR_Cat2_3000.txt 
It needs pocketsphinx to run and the Topic that it has to listen at.
It writes all accepted Commands in one file and the declined and partly parsed Commands into 2 other files.

Usage: python testGrammar.py CommandsFileName acceptedFileName declinedFileName partlyFileName Topic
