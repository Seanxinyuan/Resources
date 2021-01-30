/* =================================================================================
File name:       SMOPOS.H   (IQ version)                  
                     
Originator:	Digital Control Systems Group
			Texas Instruments

Description: 
Header file containing constants, data type definitions, and 
function prototypes for the SMOPOS.
====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2010	Version 1.1                                                 
------------------------------------------------------------------------------*/

typedef struct {  _iq  Valpha;   	// Input: Stationary alfa-axis stator voltage 
                  _iq  Ealpha;   	// Variable: Stationary alfa-axis back EMF 
                  _iq  Zalpha;      // Output: Stationary alfa-axis sliding control 
                  _iq  Gsmopos;    	// Parameter: Motor dependent control gain 
                  _iq  EstIalpha;   // Variable: Estimated stationary alfa-axis stator current 
                  _iq  Fsmopos;    	// Parameter: Motor dependent plant matrix 
                  _iq  Vbeta;   	// Input: Stationary beta-axis stator voltage 
                  _iq  Ebeta;  		// Variable: Stationary beta-axis back EMF 
                  _iq  Zbeta;      	// Output: Stationary beta-axis sliding control 
                  _iq  EstIbeta;    // Variable: Estimated stationary beta-axis stator current 
                  _iq  Ialpha;  	// Input: Stationary alfa-axis stator current 
                  _iq  IalphaError; // Variable: Stationary alfa-axis current error                 
                  _iq  Kslide;     	// Parameter: Sliding control gain 
                  _iq  Ibeta;  		// Input: Stationary beta-axis stator current 
                  _iq  IbetaError;  // Variable: Stationary beta-axis current error                 
                  _iq  Kslf;       	// Parameter: Sliding control filter gain 
                  _iq  Theta;     	// Output: Compensated rotor angle 
                 
				 } SMOPOS;	            

typedef SMOPOS *SMOPOS_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the SMOPOS object.
-----------------------------------------------------------------------------*/                     
#define SMOPOS_DEFAULTS {  0,0,0,0,0,0,0,0,0,0,   \
	                       0,0,0,0,0,0,0,   \
              			 }

/*------------------------------------------------------------------------------
Prototypes for the functions in SMOPOS.C
------------------------------------------------------------------------------*/

_iq E0 =_IQ(0.5);
_iq invE0=_IQ(2.0);

#define SMO_MACRO(v)																					\
																										\
    /*	Sliding mode current observer	*/																\
    v.EstIalpha = _IQmpy(v.Fsmopos,v.EstIalpha) + _IQmpy(v.Gsmopos,(v.Valpha-v.Ealpha-v.Zalpha));		\
    v.EstIbeta = _IQmpy(v.Fsmopos,v.EstIbeta) + _IQmpy(v.Gsmopos,(v.Vbeta-v.Ebeta-v.Zbeta));			\
																										\
	/*	Current errors	*/																				\
    v.IalphaError = v.EstIalpha - v.Ialpha;																\
    v.IbetaError= v.EstIbeta - v.Ibeta;																	\
    																									\
	/*  Sliding control calculator	*/																	\
    if (_IQabs(v.IalphaError) < E0)																		\
    	v.Zalpha = _IQmpy(v.Kslide,_IQmpy2(v.IalphaError));	/* (v.Kslide*(v.IalphaError)/E0) */			\
	else if (v.IalphaError >= E0)																		\
		v.Zalpha = v.Kslide;																			\
	else if (v.IalphaError <= -E0)																		\
		v.Zalpha = -v.Kslide;																			\
	if (_IQabs(v.IbetaError) < E0)																		\
		v.Zbeta = _IQmpy(v.Kslide,_IQmpy2(v.IbetaError));   /* (v.Kslide*(v.IbetaError)/E0) */			\
	else if (v.IbetaError >= E0)																		\
		v.Zbeta = v.Kslide;																				\
	else if (v.IbetaError <= -E0)																		\
		v.Zbeta = -v.Kslide;																			\
																										\
	/*	Sliding control filter -> back EMF calculator	*/												\
    v.Ealpha = v.Ealpha + _IQmpy(v.Kslf,(v.Zalpha-v.Ealpha));											\
    v.Ebeta = v.Ebeta + _IQmpy(v.Kslf,(v.Zbeta-v.Ebeta));												\
																										\
	/*	Rotor angle calculator -> Theta = atan(-Ealpha,Ebeta)	*/										\
	v.Theta = _IQatan2PU(-v.Ealpha,v.Ebeta); 

