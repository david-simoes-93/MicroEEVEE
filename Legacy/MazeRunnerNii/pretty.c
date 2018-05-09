#define CLR_RED 41
#define CLR_YELLOW 43
#define CLR_GREEN 42
#define CLR_BLUE 44
#define CLR_MAGENTA 45
#define CLR_CYAN 46

void printHeader(const char* str1, int clr) {
	printf("[\e[1m\e[%dm %s \e[0m]: ", clr, str1);
}
