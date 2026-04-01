#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H

#include <stdio.h>
#include <string.h>

int tests_run = 0;
int tests_failed = 0;

#define ASSERT(message, test) do { if (!(test)) { printf("FAIL: %s\n", message); tests_failed++; return 1; } } while (0)
#define RUN_TEST(test) do { printf("Running %s... ", #test); int result = test(); tests_run++; if (result) printf("PASS\n"); else printf("FAIL\n"); } while (0)

#define TEST_ASSERT_EQUAL_INT(expected, actual) do { \
    if ((expected) != (actual)) { \
        printf("FAIL: Expected %d, got %d at line %d\n", (int)(expected), (int)(actual), __LINE__); \
        tests_failed++; \
        return 0; \
    } \
} while (0)

#define TEST_ASSERT_EQUAL_PTR(expected, actual) do { \
    if ((expected) != (actual)) { \
        printf("FAIL: Expected pointer %p, got %p at line %d\n", (void*)(expected), (void*)(actual), __LINE__); \
        tests_failed++; \
        return 0; \
    } \
} while (0)

#define TEST_ASSERT_EQUAL_HEX8(expected, actual) do { \
    if ((expected) != (actual)) { \
        printf("FAIL: Expected 0x%02X, got 0x%02X at line %d\n", (unsigned int)(expected), (unsigned int)(actual), __LINE__); \
        tests_failed++; \
        return 0; \
    } \
} while (0)

#endif // TEST_FRAMEWORK_H
