from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from dbModels import Base

# For SQLite (file-based). Replace URL for other backends.
engine = create_engine('sqlite:///calibration.db', echo=True)

SessionLocal = sessionmaker(bind=engine)


def init_db(drop_first: bool = False):
    if drop_first:
        Base.metadata.drop_all(engine)
    Base.metadata.create_all(engine)

def drop_all():
    Base.metadata.drop_all(engine)
    print("✅ all tables dropped")

if __name__ == "__main__":
    init_db(drop_first=True)
    print("Database initialized.")