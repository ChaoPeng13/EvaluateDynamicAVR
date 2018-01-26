#include "Utility.h"
#include "Global.h"

double Utility::EPSILON = 0.000001;

double POS_INFINITY = std::numeric_limits<double>::infinity();
double NEG_INFINITY = - std::numeric_limits<double>::infinity();

double RPM_to_W(double rpm)
{
	return (2*M_PI)/(1.0/((rpm/60))*pow(10.0,6.0));
}

double W_to_RPM(double w)
{
	return 60*pow(10,6.0)*w/(2*M_PI);
}

double RPM_to_RPmSEC(double w)
{
	return w/(60.0*pow(10.0,3));
}

double RPmSEC_to_RPM(double w)
{
	return w*(60.0*pow(10.0,3));
}

double RPM_to_RPmuSEC(double w)
{
	return w/(60.0*pow(10.0,6));
}

double RPmuSEC_to_RPM(double w)
{
	return w*(60.0*pow(10.0,6));
}

double RPmSEC_to_HundredRPM(double w) {
	return RPmSEC_to_RPM(w) * pow(10.0,2);
}

double HundredRPM_to_RPmSEC(double w) {
	return RPM_to_RPmSEC(w/pow(10.0,2));
}

double RPmSEC_to_ThousandRPM(double w) {
	return RPmSEC_to_RPM(w) * pow(10.0,3);
}

double ThousandRPM_to_RPmSEC(double w) {
	return RPM_to_RPmSEC(w/pow(10.0,3));
}

double mSEC_to_muSEC(double T)
{
	return T*pow(10.0,3);
}

double muSEC_to_mSEC(double T)
{
	return T/pow(10.0,3);
}

// return the greatest common divisor for two integers
int Utility::math_gcd(int a, int b) {
		if (b==0) return a;
		return math_gcd(b, a%b);
}

int Utility::math_lcm(int a, int b) {
		if (b==0) return a;
		return (a*(b/math_gcd(a,b)));
}

int Utility::round(double r) {
	return (r>0.0) ? floor(r+0.5) : ceil(r-0.5);
}

std::string Utility::int_to_string(int a) {
#ifdef WINDOWS_PLATFORM
		char temp[10];
		sprintf_s(temp,"%d",a);
		return temp;
#endif

#ifdef LINUX_PLATFORM
		return std::to_string(a);
#endif
}

double** Utility::creat_matrix(int nrow, int ncol) {
	double** A = new double*[nrow];
	for (int i=0; i<nrow; i++) A[i] = new double[ncol];
	return A;
}

void Utility::output_matrix(double** A, int nrow, int ncol) {
	for (int i=0; i<nrow; i++) {
		for (int j=0; j<ncol; j++) {
			std::cout<< A[i][j];
			if (j==ncol-1) std::cout<<std::endl;
			else std::cout<<"\t";
		}
	}
}

void Utility::output_latex_matrix(double** A, int nrow, int ncol) {
	for (int i=0; i<nrow; i++) {
		std::cout << "c";
	}
	std::cout << std::endl;

	for (int i=0; i<nrow; i++) {
		for (int j=0; j<ncol; j++) {
			if (A[i][j] == NEG_INFINITY) std::cout << "-\\infty";
			else std::cout<< 0.01*A[i][j];
			if (j==ncol-1) std::cout << "\\\\" <<std::endl;
			else std::cout<<"&";
		}
	}
}

vector<double> Utility::uniformly_distributed(int n, double tUtil) {
	vector<double> ret;

	double sum = tUtil;
	for (int i=0; i<n-1; i++) {
		double nextsum = sum*pow((double)rand()/RAND_MAX, 1.0/(n-i-1));
		ret.push_back(sum-nextsum);
		sum = nextsum;
	}

	ret.push_back(sum);
	return ret;
}

vector<double> Utility::uniformly_distributed(int n, double tUtil, double minUtil) {
	vector<double> ret;

	double sum = tUtil;
	for (int i=0; i<n-1; i++) {
		double nextsum = sum*pow((double)rand()/RAND_MAX, 1.0/(n-i-1));
		if (sum - nextsum < minUtil) {
			ret.push_back(minUtil); 
			sum -= minUtil;
		}
		else {
			ret.push_back(sum-nextsum);
			sum = nextsum;
		}
	}

	ret.push_back(max(sum,minUtil));
	return ret;
}

double* Utility::uniformly_distributed2(int n, double tUtil) {
	double* ret = new double[n];

	double sum = tUtil;
	for (int i=0; i<n-1; i++) {
		double nextsum = sum*pow((double)rand()/RAND_MAX, 1.0/(n-i-1));
		if (nextsum < 0.001) nextsum = 0.001;
		nextsum = (int)(nextsum*1000);
		nextsum = nextsum/1000;
		ret[i] = sum-nextsum;
		sum = nextsum;
	}

	ret[n-1] = sum;
	return ret;
}

double Utility::max_uniformly_distributed(int n, double tUtil) {
	double* ret = new double[n];

	double sum = tUtil;
	for (int i=0; i<n-1; i++) {
		double nextsum = sum*pow((double)rand()/RAND_MAX, 1.0/(n-i-1));
		ret[i] = sum-nextsum;
		sum = nextsum;
	}

	ret[n-1] = sum;

	double max = 0;
	for (int i=0; i<n; i++) {
		if (max < ret[i]) max = ret[i];
	}

	delete[] ret;
	return max;
}

bool Utility::compare_two_matrices(double** A, double** B, int n) {
	for (int i=0; i<n; i++) for (int j=0; j<n; j++)
		if (abs(A[i][j]-B[i][j])>EPSILON) {
			if (false) {
				std::cout<<i<<","<<j<<std::endl;
				std::cout<<A[i][j]<<"..."<<B[i][j]<<std::endl;
			}
			return false;
		}
	return true;
}

void Utility::output_one_vector(ostream& out, string sVec, vector<double> vec) {
	out<<sVec<<"=[";
	typedef vector<double>::iterator Iter;
	for (Iter it = vec.begin(); it != vec.end(); it++) {
		out<<*it;
		if (it != --vec.end()) out<<",";
	}
	out<<"];"<<endl;
}

void Utility::output_one_vector(ostream& out, string sVec, vector<int> vec) {
	out<<sVec<<"=[";
	typedef vector<int>::iterator Iter;
	for (Iter it = vec.begin(); it != vec.end(); it++) {
		out<<*it;
		if (it != --vec.end()) out<<",";
	}
	out<<"];"<<endl;
}

std::string Utility::get_one_vector(string sVec, vector<double> vec)
{
	string result = sVec + "=[";
	typedef vector<double>::iterator Iter;
	for (Iter it = vec.begin(); it != vec.end(); it++) {
		result += int_to_string(round(*it));
		if (it != --vec.end()) result += ",";
	}
	result += "]";
	return result;
}

std::string Utility::get_one_vector(string sVec, vector<int> vec)
{
	string result = sVec + "=[";
	typedef vector<int>::iterator Iter;
	for (Iter it = vec.begin(); it != vec.end(); it++) {
		result += int_to_string(*it);
		if (it != --vec.end()) result += ",";
	}
	result += "]";
	return result;
}

void Utility::output_multiple_vectors(ostream& out, vector<string> sVecs, vector<vector<double>> vecs)
{
	if (sVecs.size()!=vecs.size()) {
		cerr << "Wrong size: " << sVecs.size() << "\t" << vecs.size() << endl;
		exit(EXIT_FAILURE);
	}

	for (int i=0; i<sVecs.size(); i++)
		output_one_vector(out,sVecs[i],vecs[i]);
}

string Utility::get_multiple_vectors(vector<string> sVecs, vector<vector<double>> vecs)
{
	if (sVecs.size()!=vecs.size()) {
		cerr << "Wrong size: " << sVecs.size() << "\t" << vecs.size() << endl;
		exit(EXIT_FAILURE);
	}

	string result = "";

	for (int i=0; i<sVecs.size(); i++) {
		result += get_one_vector(sVecs[i],vecs[i]);
		if (i!=sVecs.size()-1)
			result += "#";
	}
	return result;
}

void Utility::output_one_map(ostream& out, string sVec, map<int,double> mm) {
	out<<sVec<<"X=[";
	typedef map<int,double>::iterator Iter;
	for (Iter it = mm.begin(); it != mm.end(); it++) {
		out<<double(it->first)/10;
		if (it != --mm.end()) out<<",";
	}
	out<<"];"<<endl;

	out<<sVec<<"Y=[0,";
	for (Iter it = mm.begin(); it != mm.end(); it++) {
		out<<it->second;
		if (it != --mm.end()) out<<",";
	}
	out<<"];"<<endl;
}

void Utility::output_one_map(ostream& out, string sVec, map<int,int> mm) {
	out<<sVec<<"X=[";
	typedef map<int,int>::iterator Iter;
	for (Iter it = mm.begin(); it != mm.end(); it++) {
		out<<it->first;
		if (it != --mm.end()) out<<",";
	}
	out<<"];"<<endl;

	out<<sVec<<"Y=[";
	for (Iter it = mm.begin(); it != mm.end(); it++) {
		out<<it->second;
		if (it != --mm.end()) out<<",";
	}
	out<<"];"<<endl;
}

void Utility::output_one_map(ostream& out, string sVec, map<int,int> mm,int t) {
	out<<sVec<<"X=[";
	typedef map<int,int>::iterator Iter;
	for (Iter it = mm.begin(); it != mm.end(); it++) {
		if (it->first > t) break;
		out<<it->first<<",";
		//if (it != --mm.end()) out<<",";
	}
	out<<t<<"];"<<endl;


	int temp = 0;
	out<<sVec<<"Y=[";
	for (Iter it = mm.begin(); it != mm.end(); it++) {
		if (it->first > t) break;
		out<<it->second<<",";
		temp = it->second;
		//if (it != --mm.end()) out<<",";
	}
	out<<temp<<"];"<<endl;
}

void Utility::output_map_map(ostream& out, string sVec, map<double, map<int,int>> mm) {
	for (auto e : mm) {
		output_one_map(out,sVec+"_"+int_to_string(RPmSEC_to_HundredRPM(e.first)),e.second);
	}
}

void Utility::output_one_map(ostream& out, string sVec, map<double,int> mm)
{
	out<<sVec<<"X=[";
	typedef map<double,int>::iterator Iter;
	for (Iter it = mm.begin(); it != mm.end(); it++) {
		out<<it->first;
		if (it != --mm.end()) out<<",";
	}
	out<<"];"<<endl;

	out<<sVec<<"Y=[";
	for (Iter it = mm.begin(); it != mm.end(); it++) {
		out<<it->second;
		if (it != --mm.end()) out<<",";
	}
	out<<"];"<<endl;
}

int Utility::getWCET(double omega, double omegas[], int wcets[], int numMode) {
	for (int i=0; i<numMode; i++) {
		double temp = omegas[i];
		if (abs(temp - omega) <= 0.0000001) 
			return wcets[i];

		if (temp > omega) 
			return wcets[i];
	}

	return wcets[0];

	cerr << "P1:getWCET, Cannot arrive here!" << "\tOmega=" << omega << endl;
	for (int i=0; i<numMode; i++) cerr << omegas[i] << "\t";
	cerr << endl;
	exit(EXIT_FAILURE);
}

bool Utility::testSeperation(set<int> testSet, int cSep, int value) {
	for (set<int>::iterator iter = testSet.begin(); iter != testSet.end(); iter++) {
		if (abs(*iter-value) < cSep) return false;
	}
	return true;
}

bool Utility::testSeperation(vector<int> testSet, int cSep, int value) {
	for (vector<int>::iterator iter = testSet.begin(); iter != testSet.end(); iter++) {
		if (abs(*iter-value) < cSep) return false;
	}
	return true;
}

vector<int> Utility::integerUniforms(bool order,int cMin, int cMax, int cSep, int n) {
	vector<int> ret;
	int rNeq0 = rand();
	while (rNeq0 == 0) rNeq0 = rand();

	default_random_engine generator(rNeq0);
	uniform_int_distribution<int> distribution(cMin,cMax);
	vector<int> temp;
	int times = 0;
	while (temp.size() != n) {
		int r = distribution(generator);
		if (testSeperation(temp,cSep,r)) {
			temp.push_back(r);
			times = 0;
		}
		else
			times ++;
		if (times >= 100) {
			//cout << "Give up!" << endl;
			temp.clear();
		}
	}
	if (order)
		sort(temp.begin(), temp.end(), less<int>());
	else 
		sort(temp.begin(), temp.end(), greater<int>());
	
	for (auto k : temp) {
		ret.push_back(k);
	}
	return ret;
}

vector<int> Utility::integerUniforms(bool order,int cMin, int cMax, int n) {
	int rNeq0 = rand();
	while (rNeq0 == 0) rNeq0 = rand();

	default_random_engine generator(rNeq0);
	uniform_int_distribution<int> distribution(cMin,cMax);
	vector<int> temp;
	while (temp.size() != n) {
		int r = distribution(generator);
		temp.push_back(r);
	}
	if (order)
		sort(temp.begin(), temp.end(), less<int>());
	else 
		sort(temp.begin(), temp.end(), greater<int>());

	return temp;
}

vector<double> Utility::uniforms(bool order,int cMin, int cMax, int cSep, int n) {
	vector<double> ret;

	int rNeq0 = rand();
	while (rNeq0 == 0) rNeq0 = rand();

	default_random_engine generator(rNeq0);
	uniform_int_distribution<int> distribution(cMin,cMax);
	vector<int> temp;
	while (temp.size() != n) {
		int r = distribution(generator);
		if (testSeperation(temp,cSep,r))
			temp.push_back(r);
	}
	if (order)
		sort(temp.begin(), temp.end(), less<int>());
	else 
		sort(temp.begin(), temp.end(), greater<int>());
	
	for (auto k : temp) {
		ret.push_back(k);
	}
	return ret;
}

vector<double> Utility::uniforms(bool order,double cMin, double cMax, int n) {
	vector<double> ret;

	int rNeq0 = rand();
	while (rNeq0 == 0) rNeq0 = rand();

	default_random_engine generator(rNeq0);
	uniform_real_distribution<double> distribution(cMin,cMax);

	while (ret.size() != n) {
		double r = distribution(generator);
		ret.push_back(r);
	}
	if (order)
		sort(ret.begin(), ret.end(), less<double>());
	else 
		sort(ret.begin(), ret.end(), greater<double>());
	
	return ret;
}

vector<string> Utility::split(const string &s, const string &seperator){
  vector<string> result;
  typedef string::size_type string_size;
  string_size i = 0;
  
  while(i != s.size()){
    //找到字符串中首个不等于分隔符的字母；
    int flag = 0;
    while(i != s.size() && flag == 0){
      flag = 1;
      for(string_size x = 0; x < seperator.size(); ++x)
		if(s[i] == seperator[x]){
		++i;
		flag = 0;
		break;
		}
    }
    
    //找到又一个分隔符，将两个分隔符之间的字符串取出；
    flag = 0;
    string_size j = i;
    while(j != s.size() && flag == 0){
      for(string_size x = 0; x < seperator.size(); ++x)
		if(s[j] == seperator[x]){
		flag = 1;
		break;
		}
      if(flag == 0) 
		++j;
    }
    if(i != j){
      result.push_back(s.substr(i, j-i));
      i = j;
    }
  }
  return result;
}

double Utility::random(double cMin, double cMax) {
	return ((double)rand()/RAND_MAX)*(cMax-cMin)+cMin;
}

std::map<int,int> Utility::merge(map<int,int> map_a, map<int,int> map_b)
{
	if (map_a.empty()) return map_b;

	map<int,int> ret = map_a;
	for (auto e:map_b) {
		int first = e.first;
		int second = e.second;
		if (map_a.find(first)==map_a.end())
			ret[first] = second;
		else ret[first] = max(ret[first],second);
	}

	set<int> removed;

	while (true) {
		bool found = false;
		int lastValue = -1;
		for (auto e : ret) {
			int first = e.first;
			int second = e.second;
			if (removed.find(first) != removed.end()) continue;
			if (second > lastValue) lastValue = second;
			else {
				removed.insert(first);
				found = true;
			}
		}

		if (!found) break;
	}
	
	map<int,int> ret2;
	for (auto e : ret) {
		int first = e.first;
		int second = e.second;
		if (removed.find(first) != removed.end()) continue;
		
		ret2[first] = second;
	}
	return ret2;
}

std::vector<double> Utility::merge(vector<double> vec0, vector<double> vec1)
{
	vector<double> ret = vec0;
	for (auto eVec1:vec1) {
		bool isExisted = false;
		for (auto eVec0:vec0) {
			if (W_EQ_TOL(eVec0,eVec1)) {
				isExisted = true;
				break;
			}
		}
		if (!isExisted) ret.push_back(eVec1);
	}

	// SORT the dominant speeds
	sort(ret.begin(),ret.end(),greater<double>());
	return ret;
}

void Utility::insert_vector(vector<double>& vec, double e)
{
	bool isExisted = false;
	for (auto record : vec) {
		if (W_EQ_TOL(record,e)) {
			isExisted = true;
			break;
		}
	}

	if (!isExisted) vec.push_back(e);
}

void Utility::makeDir(string dirName)
{
#ifdef WINDOWS_PLATFORM
	if ( _mkdir(dirName.c_str()) == 0) {
		cout<<"Directory: "<<dirName<<" was successfully created."<<endl;
	}
#endif

#ifdef LINUX_PLATFORM
	if (mkdir(dirName.c_str(),S_IRWXU|S_IRGRP|S_IXGRP|S_IROTH) == 0) {
		cout<<"Directory: "<<dirName<<" was successfully created."<<endl;
	}
#endif
}

std::string Utility::linkNotation()
{
#ifdef WINDOWS_PLATFORM
	return "\\";
#endif

#ifdef LINUX_PLATFORM
	return "/";
#endif
}
