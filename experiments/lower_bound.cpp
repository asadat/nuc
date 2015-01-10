#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int p2(int n)
{
    int pow=1;
    for(int i=0; i<n; i++)
    {
        pow *= 2;
    }

    return pow;
}

double DiagonalLength(int level, double lower_cell_l)
{
    return sqrt(3) * lower_cell_l * (p2(level)-1)/2;
}


int main(int argc, char ** argv)
{
    double ln = 2;
    for(int p = 10; p < 100; p += 10)
    {
        double length = 0;
        double remain = 4180 * (100.0 - p)/100.0;
        for(int i=10; i>=0; i--)
        {
            if(ln*ln*p2(i)*p2(i) < remain)
            {
                remain -= ln*ln*p2(i)*p2(i);
                length += DiagonalLength(i, ln);

                i++;
            }
        }

        printf("P: %d\tL: %f\n", p, length);
    }
}
