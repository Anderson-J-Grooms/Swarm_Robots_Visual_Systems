#include <cstdlib>
#include <string>
#include <sstream>
#include <fstream>

using namespace std;

int main() {
	while(1) {
		string call_line = "echo \"Hello\" > /dev/ttyAMA0";
                system(call_line.c_str());

	}
	return 0;
}
