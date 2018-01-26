#include "FileWriter.h"

void FileWriter::WriteTaskSystem(const char* fname, vector<PeriodicTask> tasks, AVRTask avrTask) {
	ofstream fout(fname, ios::out | ios::trunc);
	if (!fout.is_open()) {
		cerr << "Can't open "<<fname<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	fout << tasks.size() << endl;
	for (int i=0; i<tasks.size(); i++)
		tasks[i].output(fout);

	avrTask.outputWCETs(fout);

	fout.close();
}

void FileWriter::WriteTaskSystem(const char* fname, vector<PeriodicTask> tasks, vector<int> wcets) {
	ofstream fout(fname, ios::out | ios::trunc);
	if (!fout.is_open()) {
		cerr << "Can't open "<<fname<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	fout << tasks.size() << endl;
	for (int i=0; i<tasks.size(); i++)
		tasks[i].output(fout);

	int numMode = wcets.size();
	fout << numMode << "\t";
	for (int i=0; i<numMode; i++) {
		fout << wcets[i];
		if (i!=numMode-1)
			fout << "\t";
	}
	fout << endl;

	fout.close();
}

void FileWriter::WriteTaskSystem(const char* fname, vector<PeriodicTask> tasks, AVRTask avrTask, int avrTaskIndex) {
	ofstream fout(fname, ios::out | ios::trunc);
	if (!fout.is_open()) {
		cerr << "Can't open "<<fname<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	fout << tasks.size() << endl;
	for (int i=0; i<tasks.size(); i++)
		tasks[i].output(fout);

	fout << avrTaskIndex << endl;
	avrTask.outputWCETs(fout);
	avrTask.outputSpeeds(fout);

	fout.close();
}

void FileWriter::WriteTaskSystem(const char* fname, vector<PeriodicTask> tasks, vector<AVRTask*> avrTasks, int avrTaskIndex) {
	ofstream fout(fname, ios::out | ios::trunc);
	if (!fout.is_open()) {
		cerr << "Can't open "<<fname<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	fout << tasks.size() << endl;
	for (int i=0; i<tasks.size(); i++)
		tasks[i].output(fout);

	fout << avrTaskIndex << endl;
	fout << avrTasks.size() << endl;
	for (auto avrTask:avrTasks) {
		avrTask->outputWCETs(fout);
		avrTask->outputSpeeds(fout);
	}

	fout.close();
}

void FileWriter::WriteTaskSystem(const char* fname, vector<AsynchronousPeriodicTask> tasks, AVRTask avrTask) {
	ofstream fout(fname, ios::out | ios::trunc);
	if (!fout.is_open()) {
		cerr << "Can't open "<<fname<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	fout << tasks.size() << endl;
	for (int i=0; i<tasks.size(); i++)
		tasks[i].output(fout);

	avrTask.outputWCETs(fout);

	fout.close();
}

void FileWriter::WriteTaskSystemLatex(const char* fname, vector<AsynchronousPeriodicTask> tasks, AVRTask avrTask) {
	ofstream fout(fname, ios::out | ios::trunc);
	if (!fout.is_open()) {
		cerr << "Can't open "<<fname<<" file for output" <<endl;
		exit(EXIT_FAILURE);
	}

	fout << tasks.size() << endl;
	for (int i=0; i<tasks.size(); i++) {
		// $\tau_i$ & priority & $c_i$ & $T_i$ & $o_i$ & $D_i$ & util\\ [0.5ex]
		AsynchronousPeriodicTask task = tasks[i];
		fout << i << " & " << i+1 << " & " 
			<< task.wcet << " & " << task.period/1000 << " & " << task.offset/1000 <<  " & " << task.deadline/1000 << " & "
			<< 1.0*task.wcet/task.period << "\\\\ \\hline" << endl;
	}

	int wcets[5] = {148,416,100,330,10};
	int sum = 0;
	for (int i=0; i<5; i++) sum += wcets[i];

	//avrTask.outputWCETs(fout);
	for (int i=avrTask.numMode-1; i>=0; i--) { 
		fout << int (9.1 * avrTask.wcets[i]); 
		if (i!=0) fout << " & ";
		else fout << "\\\\ \\hline";
	}
	fout << endl;

	// Five avr task
	for (int i=0; i<5; i++) {
		for (int j=avrTask.numMode-1; j>=0; j--) { 
			fout << int (9.1 * wcets[i] * avrTask.wcets[j]/sum); 
			if (j != 0) fout << " & ";
			else fout << "\\\\ \\hline";
		}
		fout << endl;
	}

	fout.close();
}