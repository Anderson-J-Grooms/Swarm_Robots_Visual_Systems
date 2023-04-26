#include <cstdlib>

int main() {
	while(1) {
		std::system("echo \"Hello\" > /dev/ttyAMA0 9600");
	}
	return 0;
}
