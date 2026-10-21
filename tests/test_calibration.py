import copy
import json
import math
import unittest
import calibrate_model as model


def sample(cv, tau=.0244, delay=.00225, ripple=0, brake=1):
    times = [5,10,20,30,40,60,80,120,200]
    kv, fc = .402, 106
    rows = []
    for duty in (360,720,1080,1440,1920,2400):
        for direction in (1,-1):
            v = (cv*10*duty/2400-fc)/kv
            ps = [round(v*model.response(t/1000,tau,delay) + ripple*math.sin((v*t/1000+direction*80)*math.pi/500)) for t in times]
            # 简化刹停距离：包含延迟，并可模拟弱制动。
            dist = round(v*delay + v*tau*(1-math.log(2))/brake)
            rows.append([duty,direction,cv,round(v),2047,round(v),dist,*ps])
    return model.parse_result('@CAL,'+json.dumps(dict(version=2,voltage_cv=cv,pwm_full=2400,sign=1,
        fs_mv=522,noise_cdeg=8,min_step_cdeg=32,step_ms=times,steps=rows)))


class CalibrationTest(unittest.TestCase):
    def test_model_and_user_parameters_are_separate(self):
        low,high=sample(600),sample(800)
        values,_=model.calculate(low,high)
        self.assertLess(abs(values['MOTOR_KA_NV']-9809),300)
        self.assertLess(abs(values['MOTOR_DELAY_US']-2250),300)
        self.assertEqual(values['MOTOR_KV_UV'],402)
        self.assertTrue(all(k.startswith('MOTOR_') for k in values))
        self.assertEqual(model.calculate(high,low)[0],values)
        self.assertNotIn('CFG_DEF_',model.header(values,low,high))

    def test_position_ripple_does_not_collapse_inertia(self):
        values,_=model.calculate(sample(600,ripple=15),sample(800,ripple=15))
        self.assertTrue(8300 < values['MOTOR_KA_NV'] < 11500,values)

    def test_weak_brake_reduces_capability(self):
        normal,_=model.calculate(sample(600),sample(800))
        weak,_=model.calculate(sample(600,brake=.5),sample(800,brake=.5))
        self.assertLess(weak['MOTOR_CAP_V2_DEC_DPS2'],normal['MOTOR_CAP_V2_DEC_DPS2'])

    def test_rejects_legacy_and_malformed_input(self):
        for text in ('','@CAL,{}','@CAL,{"ka_nv":1820}','@CAL,[]'):
            with self.assertRaises(ValueError): model.parse_result(text)
        bad=sample(600);bad['steps'][0][4]=1<<20
        with self.assertRaises(ValueError): model.parse_result('@CAL,'+json.dumps(bad))

    def test_rejects_missing_measurement_and_inconsistent_plant(self):
        bad=sample(800)
        for s in bad['steps']: s[4] &= ~(1<<9)
        with self.assertRaises(ValueError): model.calculate(sample(600),bad)
        with self.assertRaises(ValueError): model.calculate(sample(600,tau=.02),sample(800,tau=.04))
