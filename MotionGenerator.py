# Pos file Generator
# import sys

class MotionGenerator:
    def __init__(self):
        self.keyframes = []
        self.defaultStiffness = 0.85
        self.header = ["HY", "HP", "LSP", "LSR", "LEY", "LER", "LWY", "LHYP", "LHR", "LHP", "LKP", "LAP", "LAR", "RHR", "RHP", "RKP", "RAP","RAR","RSP","RSR","REY","RER","RWY","LH","RH", "DUR"]

    def setDefaultStiffness(self, stiffnessVal):
        self.defaultStiffness = stiffnessVal

    def addKeyframe(self, duration, jointvals, stiffnessvals=[], description=""):
        new_keyframe = {
                        "description":   description, 
                        "joint_vals":     jointvals, 
                        "stiffness_vals": stiffnessvals,
                        "duration":      duration
                        }
        self.keyframes.append(new_keyframe)

    def generatePosFile(self, filename, width=7, precision=1):
        print_width = width         # column spacing
        print_precision = precision # value precision
        
        with open(filename+".pos", 'w') as posFile:
            posFile.write('  ')
            for joint_name in self.header:
                posFile.write(f'{joint_name:<{print_width}}')
            posFile.write('\n\n')

            for keyframe in self.keyframes:
                posFile.write(f'{keyframe["description"]}\n')
                
                # If stiffness is not specified, set them to the default value defined above.
                # Or is it better to skip the stiffness values line when writing ?
                if len(keyframe["stiffness_vals"]) == 0:
                    keyframe["stiffness_vals"] = [self.defaultStiffness] * 25

                posFile.write("$ ")
                for stiffnessval in keyframe["stiffness_vals"]:
                    posFile.write(f'{stiffnessval:<{print_width}.{print_precision}f}')

                posFile.write("\n! ")
                for jointval in keyframe["joint_vals"]:
                    posFile.write(f'{jointval:<{print_width}.{print_precision}f}')
                
                posFile.write(f'{keyframe["duration"]}\n\n')
