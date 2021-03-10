#include "ap_lib.h"
#include <stddef.h>
#include <string.h>
#include <stdio.h>


uint32_t hatol(const char *str)
{
    uint32_t l;
    uint8_t b;

    if (str == NULL)
    {
		return 0;
    }

    while (*str == ' ')	/* skip leading spaces */
    {
		str++;
    }

    if (*str == '\0')
    {
		return 0;
    }

    if ((*str == '0') && ((*(str+1) == 'x') || (*(str+1) == 'X'))) {
		str += 2;
    }

    if (strlen(str) > (sizeof(uint32_t) * 2))
    {
		/* number is too big */
		return 0;
    }

    l = 0;

    for (; (*str != 0); str++)
    {
		if ((*str >= '0') && (*str <= '9')) {
		    b = *str - '0';
		}
		else if ((*str >= 'a') && (*str <= 'f')) {
		    b = (*str - 'a') + 10;
		}
		else if ((*str >= 'A') && (*str <= 'F')) {
		    b = (*str - 'A') + 10;
        }
		else
		{
//		    errno = EINVAL;
//		    return 0;
			return l;	/* by jjlee */
		}
		l = (l * 16) + b;
    }
    return l;
}

static uint32_t aatol(const char *str)
{
    uint32_t l;
    uint8_t b;

    if (str == NULL)
    {
		return 0;
    }

    while (*str == ' ')	/* skip leading spaces */
    {
		str++;
    }

    if (*str == '\0')
    {
		return 0;
    }

    if (strlen(str) > (sizeof(uint32_t) * 2))
    {
		/* number is too big */
		return 0;
    }

    l = 0;

    for (; (*str != 0); str++)
    {
		if ((*str >= '0') && (*str <= '9')) {
		    b = *str - '0';
		}
		else
		{
			return l;
		}
		l = (l * 10) + b;
    }
    return l;
}


uint32_t ap_atoi(const char *str)
{
    uint32_t result;

    if (str == NULL)
    {
        return 0;
    }

    while (*str == ' ') /* skip leading spaces */
    {
        str++;
    }

    if (*str == '\0')
    {
        return 0;
    }

    if ((*str == '0') && ((*(str+1) == 'x') || (*(str+1) == 'X'))) {
        result= hatol(str);
    }
    else {
        result= aatol(str);
    }

    return result;
}

int ap_stricmp(char* pString1, char *pString2)
{
	int nResultValue = 0;
	int nCount = 0;
	int nConvertLow1 = 0;
	int nConvertLow2 = 0;


	while (1)
	{
		nConvertLow1 = pString1[nCount];
		nConvertLow2 = pString2[nCount];
		//대문자 인지 검사해줘서 조건에 해당하면 소문자로 변환해주는 작업을해준다.
		if (nConvertLow1 >= 'A' &&  nConvertLow1 <= 'Z')
		{
			nConvertLow1 += 32;
		}

		if (nConvertLow2 >= 'A' &&  nConvertLow1 <= 'Z')
		{
			nConvertLow2 += 32;
		}

		//변환된 대상으로 검사를 진행한다.
		if (nConvertLow1 == nConvertLow2)
		{
			if (nConvertLow1 == '\0')
			{
				nResultValue = 0;
				break;
			}
		}
		else
		{
			nResultValue = nConvertLow1;
			break;
		}
		nCount++;
	}

	return nResultValue;

}

int ap_strnicmp(char* pString1, char *pString2, int cnt)
{
	int nResultValue = 0;
	int nCount = 0;
	int nConvertLow1 = 0;
	int nConvertLow2 = 0;


	for(; nCount < cnt;)
	{
		nConvertLow1 = pString1[nCount];
		nConvertLow2 = pString2[nCount];
		//대문자 인지 검사해줘서 조건에 해당하면 소문자로 변환해주는 작업을해준다.
		if (nConvertLow1 >= 'A' &&  nConvertLow1 <= 'Z')
		{
			nConvertLow1 += 32;
		}

		if (nConvertLow2 >= 'A' &&  nConvertLow1 <= 'Z')
		{
			nConvertLow2 += 32;
		}

		//변환된 대상으로 검사를 진행한다.
		if (nConvertLow1 == nConvertLow2)
		{
			if (nConvertLow1 == '\0')
			{
				nResultValue = 0;
				break;
			}
		}
		else
		{
			nResultValue = nConvertLow1;
			break;
		}
		nCount++;
	}

	return nResultValue;

}

