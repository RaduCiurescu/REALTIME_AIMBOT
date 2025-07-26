from sqlalchemy import (
  Column,
  Integer,
  String,
  Numeric,
)
from sqlalchemy.orm import declarative_base

Base = declarative_base()

class Calibration(Base):
    __tablename__ = "calibrations"

    id          = Column(Integer, primary_key=True)
    obj_name    = Column(String(255), nullable=False)
    obj_radius  = Column(Numeric(12,6), nullable=False)
    step_length = Column(Numeric(12,6), nullable=False)
    coef_A      = Column(Numeric(38,12), nullable=False)
    coef_B      = Column(Numeric(38,12), nullable=False)
    coef_C      = Column(Numeric(38,12), nullable=False)