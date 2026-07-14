#ifndef MYSTRING_H
#define MYSTRING_H
// 自定义字符串比较函数
int my_strcmp(const char *s1, const char *s2);

// 自定义字符串复制函数
void my_strncpy(char *dest, const char *src, size_t n);

// 自定义子字符串查找函数
char *my_strstr(const char *haystack, const char *needle);

// 自定义去除换行符函数
void remove_newline(char *str);

char *my_strchr(const char *str, int c);

char *my_strtok(char *str, const char *delim);

#endif