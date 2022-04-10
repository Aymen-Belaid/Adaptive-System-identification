# RLS-ALGORITHM
# 1 - Introduction
When working on a dynamic system, either to control it or to study it's stability or what ever, we find ourselves stuck not knowing its parameters. 
Scrolling down datasheets and finding out each parameter of each component of the system is a way of doing, but is it the most efficient way? 
To answer the efficency question, let's look at a brief exemple.<br/> 

>*Let's imagine a plane flying and all of a suddent one of its wings broke up or got stuck in an unexpected 70-80 Km/h wind or finds itself overloded.
You got the idea ! <br/>*

These unexpected changes impact jurasticly the dynamics of the system and in consequence the system's parameters change.<br/>
These changes are called uncertainties and in a dynamical system uncertainties happen frequently. And they always exist due to modeling errors, measurement inaccuracy, mutations in the evolutionary processes, environnment variations and so on. </br>
In order to work with a system with structured and unstructured uncertainties we have to modelise it in an recursive way so it changes and adapts to dynamics changes. 
A way to do so is the **Recursive Least Square algorithm** [RLS]for system's parameter identification. This estimation theory refers to calculating the coefficients of a model from a set of inputs and outputs.
The goal of this algorithm is to estimate the coefficients of the transfer function of the system given the inputs and output as shown in the block diagram down below :
~~~
                                                      _ _ _ _ _ _ _ _ _ _ _ _ 
                                  Model input        |                        |   Model Output
                                  ------------------>| Model to be estimated  |--------------------->
                                    |                |_ _ _ _ _ _ _ _ _ _ _ _ |        |
                                    |        _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ |    
                                    |       |        _ _ _ _ _ _ _ _ _ _ _ _ _ 
                                    |        ------>|                         |  Estimated output
                                    |-------------->|         Estimator       |---------------------->
                                                    |_ _ _ _ _ _ _ _ _ _ _ _ _| 

~~~
# 2 - Theory and formulation of the RLS

                   
                   
                   
                   
