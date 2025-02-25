# Nonlinear Model Predictive Control on Resource-Constrained Microcontrollers
We are considering the NMPC with the following formulation:
<img src="figures/formu1.png" width="1000">
To approximate the nonlinear dynamical function, we use its Jacobian at each time step:
<img src="figures/formu2.png" width="500">
where: 
<img src="figures/formu3.png" width="1000">
Thus, the linearized model can be rewritten as:
<img src="figures/formu4.png" width="1000">
which simplifies to:
<img src="figures/formu5.png" width="1000">
Hence, the nonlinear dynamical model can be approximated at each time step by:
<img src="figures/formu6.png" width="1000">
Then the NMPC becomes the QP problem as: 
<img src="figures/formu7.png" width="1000">
