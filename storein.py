from db import SessionLocal
from models import Calibration


def store_calibration(
        obj_name: str,
        obj_radius: float,
        step_length: float,
        coef_A: float,
        coef_B: float,
        coef_C: float
):
    session = SessionLocal()
    cal = Calibration(
        obj_name=obj_name,
        obj_radius=obj_radius,
        step_length=step_length,
        coef_A=coef_A,
        coef_B=coef_B,
        coef_C=coef_C
    )
    session.add(cal)
    session.commit()
    session.close()
    print(f"Stored calibration id={cal.id}")


# Example
if __name__ == "__main__":
    store_calibration(
        obj_name="BLUE",
        obj_radius=21,
        step_length=5,
        coef_A=0.0855,
        coef_B=-13.33,
        coef_C=621.4
    )