#include <gtest/gtest.h>

int main() {
	int argc = 1;
	char* argv[] = {""};

	::testing::InitGoogleTest(&argc, argv);

	printf("Hello\r\n");

	return RUN_ALL_TESTS();
}
