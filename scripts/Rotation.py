def forwarding_orientation(x):
    a1 = x[0]
    a2 = x[2]
    a3 = x[4]
    b1 = x[1]
    b2 = x[3]
    b3 = x[5]
    rotation_3 = np.array(
        [[-(1 - cos(a3)) * (-(1 - cos(a1)) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b1) * cos(b1) - (1 - cos(a2)) * (
                    -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(b2) * cos(
            b1)) * sin(b3) * cos(b3) + (-(1 - cos(a3)) * cos(b3) ** 2 + 1) * (
                      (1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                          -(1 - cos(a1)) * cos(b1) ** 2 + 1) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) - sin(a1) * sin(
                  a2) * cos(b1) * cos(b2)) - (-(1 - cos(a1)) * sin(a2) * sin(b1) * sin(b2) * cos(b1) + (
                    -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(a2) * cos(b2) + sin(a1) * cos(a2) * cos(b1)) * sin(
            a3) * cos(b3),
          - (1 - cos(a3)) * ((1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                    -(1 - cos(a1)) * cos(b1) ** 2 + 1) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) - sin(a1) * sin(a2) * cos(
            b1) * cos(b2)) * sin(b3) * cos(b3) + (-(1 - cos(a3)) * sin(b3) ** 2 + 1) * (
                      -(1 - cos(a1)) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b1) * cos(b1) - (1 - cos(a2)) * (
                          -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(b2) * cos(
                  b1)) - (-(1 - cos(a1)) * sin(a2) * sin(b1) * sin(b2) * cos(b1) + (
                    -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(a2) * cos(b2) + sin(a1) * cos(a2) * cos(b1)) * sin(
            a3) * sin(b3),
          (-(1 - cos(a1)) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(b1) * cos(b1) - (1 - cos(a2)) * (
                      -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(b2) * cos(
              b1)) * sin(a3) * sin(b3) + (-(1 - cos(a1)) * sin(a2) * sin(b1) * sin(b2) * cos(b1) + (
                    -(1 - cos(a1)) * cos(b1) ** 2 + 1) * sin(a2) * cos(b2) + sin(a1) * cos(a2) * cos(b1)) * cos(a3) + (
                      (1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                          -(1 - cos(a1)) * cos(b1) ** 2 + 1) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) - sin(a1) * sin(
                  a2) * cos(b1) * cos(b2)) * sin(a3) * cos(b3)],
         [-(1 - cos(a3)) * ((1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                        -(1 - cos(a1)) * sin(b1) ** 2 + 1) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) - sin(a1) * sin(
                a2) * sin(b1) * sin(b2)) * sin(b3) * cos(b3) + (-(1 - cos(a3)) * cos(b3) ** 2 + 1) * (
                      -(1 - cos(a1)) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(b1) * cos(b1) - (1 - cos(a2)) * (
                          -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(b1) * cos(
                  b2)) - (-(1 - cos(a1)) * sin(a2) * sin(b1) * cos(b1) * cos(b2) + (
                        -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(a2) * sin(b2) + sin(a1) * sin(b1) * cos(a2)) * sin(
                a3) * cos(b3),
          - (1 - cos(a3)) * (
                      -(1 - cos(a1)) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(b1) * cos(b1) - (1 - cos(a2)) * (
                          -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(b1) * cos(
                  b2)) * sin(b3) * cos(b3) + (-(1 - cos(a3)) * sin(b3) ** 2 + 1) * (
                      (1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                          -(1 - cos(a1)) * sin(b1) ** 2 + 1) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) - sin(a1) * sin(
                  a2) * sin(b1) * sin(b2)) - (-(1 - cos(a1)) * sin(a2) * sin(b1) * cos(b1) * cos(b2) + (
                        -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(a2) * sin(b2) + sin(a1) * sin(b1) * cos(a2)) * sin(
                a3) * sin(b3),
          (-(1 - cos(a1)) * (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(b1) * cos(b1) - (1 - cos(a2)) * (
                      -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(b2) * cos(b2) - sin(a1) * sin(a2) * sin(b1) * cos(
              b2)) * sin(a3) * cos(b3) + (-(1 - cos(a1)) * sin(a2) * sin(b1) * cos(b1) * cos(b2) + (
                        -(1 - cos(a1)) * sin(b1) ** 2 + 1) * sin(a2) * sin(b2) + sin(a1) * sin(b1) * cos(a2)) * cos(
                a3) + ((1 - cos(a1)) * (1 - cos(a2)) * sin(b1) * sin(b2) * cos(b1) * cos(b2) + (
                        -(1 - cos(a1)) * sin(b1) ** 2 + 1) * (-(1 - cos(a2)) * sin(b2) ** 2 + 1) - sin(a1) * sin(
                a2) * sin(b1) * sin(b2)) * sin(a3) * sin(b3)],
         [-(1 - cos(a3)) * ((1 - cos(a2)) * sin(a1) * sin(b2) * cos(b1) * cos(b2) - (
                        -(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(a1) * sin(b1) - sin(a2) * sin(b2) * cos(a1)) * sin(
                b3) * cos(b3) + (-(1 - cos(a3)) * cos(b3) ** 2 + 1) * (
                      (1 - cos(a2)) * sin(a1) * sin(b1) * sin(b2) * cos(b2) - (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(
                  a1) * cos(b1) - sin(a2) * cos(a1) * cos(b2)) - (
                      -sin(a1) * sin(a2) * sin(b1) * sin(b2) - sin(a1) * sin(a2) * cos(b1) * cos(b2) + cos(a1) * cos(
                  a2)) * sin(a3) * cos(b3),
          - (1 - cos(a3)) * (
                      (1 - cos(a2)) * sin(a1) * sin(b1) * sin(b2) * cos(b2) - (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(
                  a1) * cos(b1) - sin(a2) * cos(a1) * cos(b2)) * sin(b3) * cos(b3) + (
                      -(1 - cos(a3)) * sin(b3) ** 2 + 1) * (
                      (1 - cos(a2)) * sin(a1) * sin(b2) * cos(b1) * cos(b2) - (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(
                  a1) * sin(b1) - sin(a2) * sin(b2) * cos(a1)) - (
                      -sin(a1) * sin(a2) * sin(b1) * sin(b2) - sin(a1) * sin(a2) * cos(b1) * cos(b2) + cos(a1) * cos(
                  a2)) * sin(a3) * sin(b3),
          (-sin(a1) * sin(a2) * sin(b1) * sin(b2) - sin(a1) * sin(a2) * cos(b1) * cos(b2) + cos(a1) * cos(a2)) * cos(
                a3) + (
                      (1 - cos(a2)) * sin(a1) * sin(b1) * sin(b2) * cos(b2) - (-(1 - cos(a2)) * cos(b2) ** 2 + 1) * sin(
                  a1) * cos(b1) - sin(a2) * cos(a1) * cos(b2)) * sin(a3) * cos(b3) + (
                      (1 - cos(a2)) * sin(a1) * sin(b2) * cos(b1) * cos(b2) - (-(1 - cos(a2)) * sin(b2) ** 2 + 1) * sin(
                  a1) * sin(b1) - sin(a2) * sin(b2) * cos(a1)) * sin(a3) * sin(b3)]]
    )
    return rotation_3