package org.firstinspires.ftc.teamcode.Saransh;

/**
 * Created by Saransh on 2/16/2017.
 */

public class Kolmogorov_Zurbenko_Filter {
/*
    long kza_coefficients[][19] = {
        {0,0,0,0,0,0,1,1,1,1,1,1,1,0,0,0,0,0,0},
        {0,0,0,1,2,3,4,5,6,7,6,5,4,3,2,1,0,0,0},
        {1,3,6,10,15,21,28,33,36,37,36,33,28,21,15,10,6,3,1}};

    final int KZA_LENGTH = 7; /* Window size may 3, 5, or 7 */
 /*   final int KZA_MAX = 4;

    final int KZA_HISTORY_LENGTH = ((KZA_LENGTH-1)*KZA_MAX);
    final int KZA_MID = (KZA_HISTORY_LENGTH)/2;

    int [] ms_init()
    {
        int [] array_KZA = new int [KZA_LENGTH];
        for (int i = 0; i < KZA_LENGTH; i++)
        {
            array_KZA[i] = 0;
        }

        return array_KZA;
    }

    int kza_filter(int current_value, int history_KZA[])
    {
        int divisor=1;
        int updated_value=0;
        int i,k;

        for(i=1;i<KZA_HISTORY_LENGTH;i++)
        {
            history_KZA[i-1]=history_KZA[i];
        }
        history_KZA[KZA_HISTORY_LENGTH-1]=current_value;

        for(k=1;k<=3;k++)
        {
            for(i=-k*(KZA_LENGTH-1)/2;i<=k*(KZA_LENGTH-1)/2;i++)
            {
                updated_value+=history_KZA[i+KZA_MID]*kza_coefficients[k-1][i+KZA_MID];
            }
            divisor*=KZA_LENGTH;
            updated_value/=divisor;
            history_KZA[KZA_MID]=updated_value;

            updated_value=0;
        }

        return history_KZA[KZA_MID];
    }
*/
}
