/*
 * Copyright © 2021 Embedded Artistry LLC.
 * See LICENSE file for licensing information.
 */

#include <cstdio>
#include <driver.hpp>

int main(void)
{
	const int expected = 42;
	int val = ret42();

	printf("Generated value: %d\n", val);

	return (expected != val);
}
