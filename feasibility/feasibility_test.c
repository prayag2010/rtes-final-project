#include <math.h>
#include <stdio.h>
#define U32_T unsigned int

#define TRUE 1
#define FALSE 0


int completion_time_feasibility(U32_T numServices, U32_T *period, U32_T *wcet, U32_T *deadline)
{
  int i, j;
  U32_T an, anext;
  
  // assume feasible until we find otherwise
  int set_feasible=TRUE;
   
  //printf("numServices=%d\n", numServices);
  
  for (i=0; i < numServices; i++)
  {
       an=0; anext=0;
       
       for (j=0; j <= i; j++)
       {
           an+=wcet[j];
       }
       
	   //printf("i=%d, an=%d\n", i, an);

       while(1)
       {
             anext=wcet[i];
	     
             for (j=0; j < i; j++)
                 anext += ceil(((double)an)/((double)period[j]))*wcet[j];
		 
             if (anext == an)
                break;
             else
                an=anext;

			 //printf("an=%d, anext=%d\n", an, anext);
       }
       
	   //printf("an=%d, deadline[%d]=%d\n", an, i, deadline[i]);

       if (an > deadline[i])
       {
          set_feasible=FALSE;
       }
  }
  
  return set_feasible;
}

int scheduling_point_feasibility(U32_T numServices, U32_T * period, U32_T * wcet, U32_T * deadline)
{
   int rc = TRUE, i, j, k, l, status, temp;

   for (i=0; i < numServices; i++) // iterate from highest to lowest priority
   {
      status=0;

      for (k=0; k<=i; k++) 
      {
          for (l=1; l <= (floor((double)period[i]/(double)period[k])); l++)
          {
               temp=0;

               for (j=0; j<=i; j++) temp += wcet[j] * ceil((double)l*(double)period[k]/(double)period[j]);

               if (temp <= (l*period[k]))
			   {
				   status=1;
				   break;
			   }
           }
           if (status) break;
      }
      if (!status) rc=FALSE;
   }
   return rc;
}

int main()
{
    //(U32_T numServices, U32_T period[],U32_T wcet[], U32_T deadline[]
    U32_T period[]={80,100,3000,10000};
    U32_T wcet[]={74,1,1,1};
    U32_T numServices=4;
    U32_T deadline[]={80,100,3000,10000};
    int i=scheduling_point_feasibility(numServices,period,wcet,deadline);
    int j= completion_time_feasibility(numServices,period,wcet,deadline);
    if(i)
        printf("pass scheduling\n");
    else
    printf("scheduling fail");
    
    if(j)
        printf("pass completion\n");
    else
    printf("completion fail");
    
}