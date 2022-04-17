# Adaptive-System-Parameter-Identification
## 1 - Introduction
When working on a dynamic system, either to control it or to study its stability or what ever, we find ourselves stuck not knowing its parameters. 
Scrolling down datasheets and finding out each parameter of each component of the system is a way of doing, but is it the most efficient way? 
To answer the efficency question, let's look at a brief exemple.<br/> 

>*Let's imagine a plane flying and all of a suddent one of its wings broke up or got stuck in an unexpected 70-80 Km/h wind or finds itself overloded.
You got the idea ! <br/>*

These unexpected changes impact jurasticly the dynamics of the system and in consequence the system's parameters change.<br/>
These changes are called uncertainties and in a dynamical system uncertainties happen frequently. And they always exist due to modeling errors, measurement inaccuracy, mutations in the evolutionary processes, environnment variations and so on. </br>
In order to work with a system with structured and unstructured uncertainties we have to modelise it in a recursive way so it changes and adapts to dynamics changes. 
A way to do so is the **Recursive Least Square algorithm** (RLS) for system's parameter identification. This estimation theory refers to calculating the coefficients of a model from a set of inputs and outputs.
The goal of this algorithm is to estimate the coefficients of the transfer function of the system given the inputs and output as shown in the block diagram down below :
~~~
                                                  _ _ _ _ _ _ _ _ _ _ _ _ 
                              Model input U(k)   |                       |   Model Output Y(k)
                              ------------------>| Model to be estimated |------------------------->
                                |                |_ _ _ _ _ _ _ _ _ _ _ _|         |
                                |        _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ |    
                                |       |        _ _ _ _ _ _ _ _ _ _ _ _ _ 
                                |        ------>|                         |  Estimated output Y'(k)
                                 -------------->|         Estimator       |------------------------->
                                                |_ _ _ _ _ _ _ _ _ _ _ _ _| 

~~~
## 2 - Theory and formulation of the RLS
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
>with:<br/>
>- **n**: The system Order 
>- **N**:The number of samples at t=N.Ts *(Ts = 1/fs sample time)*

With these definitions and after mathematical developpement we define a recursive algorithm that gets updated each sample time with a set of inputs and outputs and generates an adaptive transfer function at each Ts.
We define a correlation matrix related to the covariance of **Theta**
>![Pn+1](https://latex.codecogs.com/png.latex?%5Cdpi%7B100%7D%20%5Cfn_cm%20%5Csmall%20%7B%5Ccolor%7BCadetBlue%7D%20P_%7BN&plus;1%7D%3D%5Cfrac%7BP_%7BN%7D%7D%7B%5Clambda%7D.%28%5Cfrac%7BI-X_%7BN&plus;1%7DX_%7BN&plus;1%7D%5E%7BT%7DP_%7BN%7D%7D%7B%5Clambda%20&plus;%20X_%7BN&plus;1%7D%5E%7BT%7DP_%7BN%7D%7D%29%7D)<br/>

A varying gain K:
>![K](https://latex.codecogs.com/png.latex?%5Cdpi%7B100%7D%20%5Cfn_cm%20%5Csmall%20%7B%5Ccolor%7BCadetBlue%7D%20K_%7BN&plus;1%7D%3D%20%5Cfrac%7BP_%7BN%7DX_%7BN&plus;1%7D%7D%7B%5Clambda%20&plus;X_%7BN&plus;1%7D%5E%7BT%7DP_%7BN%7DX_%7BN&plus;1%7D%7D%7D)

thus we define the updated transfer function coefficients:
>![Theta](https://latex.codecogs.com/png.latex?%5Cdpi%7B100%7D%20%5Cfn_cm%20%5Csmall%20%7B%5Ccolor%7BCadetBlue%7D%20%5CTheta%20_%7BN&plus;1%7D%20%3D%20%5CTheta_%7BN%7D%20&plus;K_%7BN&plus;1%7D%5BY_%7BN&plus;1%7D%20-%20X_%7BN&plus;1%7D%5E%7BT%7D%5CTheta%20_%7BN%7D%5D%7D)

With the recursive equations in place, I will give a brieve description of the implementation of RLS algorithm on a NUCLEO-STM32H723ZG board. 
And testing it's performance on a DC Motor.<br/>
## 3 - RLS implementation and Testing
I will present in this section the steps I have done to validate and implement RLS Algorithm.
I first started by validating the model on Matlab because it was more practical in the implementation and the fastest way to test the efficency of my RLS algotithm.
So I collected Input (Voltage) and Output (Speed) data from the Microcontroller using semi-hosting feature.<br/>
The input signal:
>![Input](https://latex.codecogs.com/png.latex?%5Cdpi%7B100%7D%20%5Cfn_cm%20%5Csmall%20%5Ccolor%7BTeal%7DInput%20%3D%204x%28%5Cfrac%7B2.2sin%283.123t&plus;1.1%29&plus;2.3%7D%7B4.9%7D&plus;%5Cfrac%7B1.7sin%285.73t&plus;0.7%29&plus;1.8%7D%7B3.5%7D&plus;%5Cfrac%7B1.4sin%282.39t&plus;1.6%29&plus;1.4%7D%7B2.8%7D%29)<br/>
>- t = N/Fs.
>- N:Number of samples at t and Fs:Sampling Frequency.

I aimed to choose the input signal as a combination of sinusoids in order to have a diverse set of data for testing.<br/>
Since I am testing the model on a DC-MOTOR with NUCLEO-STM32H723ZG board, I will be using the PWM Timer's feature to generate this Input signal with varying the Duty Cycle as shown below : 
><img src="https://www.electronique-mixte.fr/wp-content/uploads/2016/01/signal-pwm-1.png" width="500" height="300">

The DC-MOTOR is equiped with an encoder so we can either measure the motor speed using the Timer's Input Capture feature or its position using Timer's encoder Mode feature. In our case we will be modelling The transfer function ***H(z)*** of the DC-MOTOR with the voltage delivered to the motor as an input and the speed (Measured) as an output.

Here I present an exemple of the algorithm's response with ***1000 samples*** and ***Fs = 100 Hz*** on the DC-MOTOR:
>*The plotted image down bellow represents the **measured output response** compared to the **simulated output response** (Simulated by the estimation algorithm).*<br/>  
><img src="https://user-images.githubusercontent.com/88536804/163709200-e3ff48a4-e620-4c16-bb35-1ba96f3c64eb.png" width="600" height="400"><br/>

- RLS_Data_Acqu folder contains the algorithm of input and output data acquisition using semi-hosting feature.<br/>
- The RLS_DC folder contains the implementation of the algorithm on the NUCLEO-STM32H723ZG board.<br/>
- Matlab_RLS folder contains data files and the implementation of the algorithm on Matlab.<br/>








                   
                   
                  
