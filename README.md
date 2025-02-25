# NMPCM : Nonlinear Model Predictive Control on Resource-Constrained Microcontrollers
We are considering the NMPC with the following formulation:
<img src="figures/formu1.png" width="1000">
To approximate the nonlinear dynamical function, we use its Jacobian at each time step:

<img src="figures/formu2.png" width="265">

where:

<img src="figures/formu3.png" width="400">

Thus, the linearized model can be rewritten as:

<img src="figures/formu4.png" width="450">

which simplifies to:

<img src="figures/formu5.png" width="450">

Hence, the nonlinear dynamical model can be approximated at each time step by:

<img src="figures/formu6.png" width="220">

Then the NMPC becomes the QP problem as: 

<img src="figures/formu7.png" width="575">

Then, in order to solve the NMPC, we used the RK4 integration method and the qpOASES solver to solve the QP problem. Utilizing the code generation of ACADO and the qpOASES library, the generated code was then embedded into the microcontroller to solve the NMPC in real time.


# Code Generation
The code generation is based on ACADO code generation and qpOASES solver, therefore the following library is required:
- ACADO Toolkit (https://acado.github.io/)
- qpOASES (https://github.com/coin-or/qpOASES)

# Contact
- [Van Chung Nguyen](mailto:vanchungn@.unr.edu)
- [Hung La](mailto:hla@unr.edu)
