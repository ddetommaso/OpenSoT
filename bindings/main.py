import yarp
import pydynutils
import pyOpenSoT

if __name__ == "__main__":
    robot = pydynutils.iDynUtils("coman",
                                 "../tests/robots/coman/coman.urdf",
                                 "../tests/robots/coman/coman.srdf")

    q = robot.iDyn3_model.getAng()

    c = pyOpenSoT.Cartesian("test_cartesian",
                            q,
                            robot,
                            "r_sole",
                            "l_sole")