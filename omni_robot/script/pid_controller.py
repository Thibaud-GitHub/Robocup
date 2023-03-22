#! /usr/bin/env python3

class PID_Controller:
    def __init__(self):
        self.Kp = 0.1
        self.Ki = 0.01
        self.Kd = 0.01
        self.integral = 0
        self.previous_error = 0

    def run(self, error):
        # Calculer la partie proportionnelle
        P = self.Kp * error
        
        # Calculer la partie intégrale
        self.integral += error
        I = self.Ki * self.integral
        
        # Calculer la partie dérivée
        D = self.Kd * (error - self.previous_error)
        self.previous_error = error
        
        # Calculer la sortie
        output = P + I + D
        
        return output