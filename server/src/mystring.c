#include <stddef.h>
// 自定义字符串比较函数
int my_strcmp(const char *s1, const char *s2) {
    while (*s1 && (*s1 == *s2)) {
        s1++;
        s2++;
    }
    return *(unsigned char *)s1 - *(unsigned char *)s2;
}

// 自定义字符串复制函数
void my_strncpy(char *dest, const char *src, size_t n) {
    size_t i;
    for (i = 0; i < n && src[i] != '\0'; i++) {
        dest[i] = src[i];
    }
    for (; i < n; i++) {
        dest[i] = '\0';
    }
}

// 自定义子字符串查找函数
char *my_strstr(const char *haystack, const char *needle) {
    if (!*needle) return (char *)haystack;
    for (; *haystack; haystack++) {
        const char *h = haystack, *n = needle;
        while (*h && *n && *h == *n) {
            h++;
            n++;
        }
        if (!*n) return (char *)haystack;
    }
    return NULL;
}

// 自定义去除换行符函数
void remove_newline(char *str) {
    for (int i = 0; str[i]; i++) {
        if (str[i] == '\n') {
            str[i] = '\0';
            break;
        }
    }
}

char *my_strchr(const char *str, int c) {
    while (*str) {
        if (*str == (char)c) {
            return (char *)str;
        }
        str++;
    }
    if ((char)c == '\0' && *str == '\0') {
        return (char *)str;
    }
    return NULL;
}

char *my_strtok(char *str, const char *delim) {
    static char *next = NULL;
    if (str != NULL) {
        next = str; // 第一次调用，初始化
    }
    if (next == NULL) {
        return NULL; // 无更多 token
    }

    // 跳过前导分隔符
    while (*next) {
        int is_delim = 0;
        for (const char *d = delim; *d; d++) {
            if (*next == *d) {
                is_delim = 1;
                break;
            }
        }
        if (!is_delim) break;
        next++;
    }
    if (*next == '\0') {
        next = NULL;
        return NULL;
    }

    char *start = next;
    while (*next) {
        int is_delim = 0;
        for (const char *d = delim; *d; d++) {
            if (*next == *d) {
                is_delim = 1;
                break;
            }
        }
        if (is_delim) break;
        next++;
    }
    if (*next) {
        *next = '\0'; // 替换为字符串结束符
        next++;
    } else {
        next = NULL;
    }
    return start;
}