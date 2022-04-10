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
A way to do so is the **Recursive Least Square algorithm** (RLS) for system's parameter identification. This estimation theory refers to calculating the coefficients of a model from a set of inputs and outputs.
The goal of this algorithm is to estimate the coefficients of the transfer function of the system given the inputs and output as shown in the block diagram down below :
~~~
                                                  _ _ _ _ _ _ _ _ _ _ _ _ 
                              Model input U(k)   |                       |   Model Output Y(k)
                              ------------------>| Model to be estimated  |------------------------->
                                |                |_ _ _ _ _ _ _ _ _ _ _ _ |        |
                                |        _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ |    
                                |       |        _ _ _ _ _ _ _ _ _ _ _ _ _ 
                                |        ------>|                         |  Estimated output Y'(k)
                                |-------------->|         Estimator       |------------------------->
                                                |_ _ _ _ _ _ _ _ _ _ _ _ _| 

~~~
# 2 - Theory and formulation of the RLS
RLS method for system identification is a least square method so its main idea consists of minimizing the *cost function* **J** as the sum of squares of the error terms.
>![erreur](https://latex.codecogs.com/png.latex?%5Cdpi%7B100%7D%20%5Cfn_cm%20%5Csmall%20%7B%5Ccolor%7BCadetBlue%7D%20e%28k%29%3DY%28k%29-Y%27%28k%29%20%7D)<br/>
>![cast function](https://latex.codecogs.com/png.latex?%5Cdpi%7B100%7D%20%5Cfn_cm%20%5Csmall%20%7B%5Ccolor%7BCadetBlue%7D%20J%3D%5Cfrac%7B1%7D%7BN%7D%5Csum_%7Bk%3D1%7D%5E%7BN%7De%28k%29%5E%7B2%7D%7D)<br/>
>N : The number of samples

We define the estimated transfer function Y'(k) as follows :
> ![Transfer function](https://latex.codecogs.com/png.latex?%5Cdpi%7B80%7D%20%5Cfn_cm%20%5Csmall%20%7B%5Ccolor%7BCadetBlue%7D%20H%28z%29%20%3D%20%5Cfrac%7BY%27%28z%29%7D%7BU%28z%29%7D%20%3D%20%5Cfrac%7Bb_%7B1%7Dz%5E%7B-1%7D%20&plus;%20b_%7B2%7Dz%5E%7B-2%7D&plus;...&plus;b_%7Bn%7Dz%5E%7B-n%7D%7D%7B1&plus;a_%7B1%7Dz%5E%7B-1%7D%20&plus;%20a_%7B2%7Dz%5E%7B-2%7D&plus;...&plus;a_%7Bn%7Dz%5E%7B-n%7D%7D%7D)<br/>

We can then write
>![Y(k)](https://latex.codecogs.com/png.latex?%5Cdpi%7B100%7D%20%5Cfn_cm%20%5Csmall%20%7B%5Ccolor%7BCadetBlue%7D%20Y%28k%29%3D%5CTheta%20X%7D)

with<br/>
>![teta](https://latex.codecogs.com/png.latex?%5Cdpi%7B100%7D%20%5Cfn_cm%20%5Csmall%20%7B%5Ccolor%7BCadetBlue%7D%20%5CTheta%5E%7BT%7D%20%3D%5Cbegin%7Bbmatrix%7D%20a_%7B1%7D%26a_%7B2%7D%26.%26.%26.%26a_%7Bn%7D%26b_%7B1%7D%26.%26.%26.%26b_%7Bn%7D%20%5Cend%7Bbmatrix%7D%7D)<br/>
>![X](https://latex.codecogs.com/png.latex?%5Cdpi%7B100%7D%20%5Cfn_cm%20%5Csmall%20%7B%5Ccolor%7BCadetBlue%7D%20X_%7Bk%7D%3D%5Cbegin%7Bbmatrix%7D%20X%5E%7BT%7D_%7B1%7D%26.%26.%26.%26X%5E%7BT%7D_%7BN%7D%20%5Cend%7Bbmatrix%7D%7D)<br/>
>![Xk](https://latex.codecogs.com/png.latex?%5Cdpi%7B100%7D%20%5Cfn_cm%20%5Csmall%20%7B%5Ccolor%7BCadetBlue%7D%20X_%7Bk%7D%3D%5Cbegin%7Bbmatrix%7D%20-Y%28k-1%29%26.%26.%26.%26-Y%28k-n%29%26U%28k-1%29%26.%26.%26.%26U%28k-n%29%20%5Cend%7Bbmatrix%7D%7D)<br/>
>with **n**: The system Order and **N**:The number of samples at t=N.Ts *(Ts = 1/fs sample time)*

With these definitions and after mathematical developpement we define a recursive algorithm that gets updated each sample time with a set of inputs and outputs and generates an adaptive transfer function at each Ts.
We define a correlation matrix related to the covariance of **Theta**
>![Pn+1](https://latex.codecogs.com/png.latex?%5Cdpi%7B100%7D%20%5Cfn_cm%20%5Csmall%20%7B%5Ccolor%7BCadetBlue%7D%20P_%7BN&plus;1%7D%3D%5Cfrac%7BP_%7BN%7D%7D%7B%5Clambda%7D.%28%5Cfrac%7BI-X_%7BN&plus;1%7DX_%7BN&plus;1%7D%5E%7BT%7DP_%7BN%7D%7D%7B%5Clambda%20&plus;%20X_%7BN&plus;1%7D%5E%7BT%7DP_%7BN%7D%7D%29%7D)<br/>

A varying gain K:
>![K](https://latex.codecogs.com/png.latex?%5Cdpi%7B100%7D%20%5Cfn_cm%20%5Csmall%20%7B%5Ccolor%7BCadetBlue%7D%20K_%7BN&plus;1%7D%3D%20%5Cfrac%7BP_%7BN%7DX_%7BN&plus;1%7D%7D%7B%5Clambda%20&plus;X_%7BN&plus;1%7D%5E%7BT%7DP_%7BN%7DX_%7BN&plus;1%7D%7D%7D)
thus we define the updated transfer function coefficients:
>![Theta](https://latex.codecogs.com/png.latex?%5Cdpi%7B100%7D%20%5Cfn_cm%20%5Csmall%20%7B%5Ccolor%7BCadetBlue%7D%20%5CTheta%20_%7BN&plus;1%7D%20%3D%20%5CTheta_%7BN%7D%20&plus;K_%7BN&plus;1%7D%5BY_%7BN&plus;1%7D%20-%20X_%7BN&plus;1%7D%5E%7BT%7D%5CTheta%20_%7BN%7D%5D%7D)




                   
                   
                   
