import os
import re
import subprocess
import time
import shutil
from pathlib import Path
from loguru import logger
from git import Repo
import wmi

DRIVERSTATION_LOGS_DIRECTORY = Path("C:/Users/Public/Documents/FRC/Log Files")
DRIVERSTATION_FILE_EXTENSION = ".dsevents"
LOG_FILE_EXTENSION = "wpilog"
LOGS_DIR = Path(os.getenv("LOGS_DIR"))
DOWNLOAD_COMPETITION_LOGS = bool(os.getenv("DOWNLOAD_COMPETITION_LOGS"))


def open_latest_log_in_advantage_scope():
    try:
        log_files = [(f, f.stat().st_mtime) for f in LOGS_DIR.glob("*.wpilog")]

        if not log_files:
            logger.warning("No log files found")
            return

        latest_log = max(log_files, key=lambda x: x[1])[0]

        logger.info(f"Opening {latest_log.name} with AdvantageScope")
        subprocess.Popen(("start", str(latest_log)))

    except Exception as error:
        logger.error(f"Failed to open log file: {error}")


def commit_and_push_log(repo, log_file):
    try:
        repo.index.add(Path(os.getcwd()) / log_file.name)
        repo.index.commit(f"Add log file: {log_file.name}")
        logger.info(f"Committed {log_file.name} to repository")

        try:
            repo.remote().push()
            logger.info(f"Successfully pushed commit for {log_file.name}")
        except Exception as error:
            logger.error(f"Error pushing commit for {log_file.name}: {error}")
    except Exception as error:
        logger.exception(f"Error committing {log_file.name}: {error}")


def get_file_signature(file_path):
    return file_path.name, file_path.stat().st_size


def is_file_downloaded(log_file):
    for existing_file in LOGS_DIR.glob(f"*.{LOG_FILE_EXTENSION}"):
        if get_file_signature(existing_file) == get_file_signature(log_file):
            return True
    return False


def get_usb_drives():
    drives = set()
    wmi_connection = wmi.WMI()

    for drive in wmi_connection.Win32_LogicalDisk():
        if drive.DriveType == 2:  # removable
            drives.add(Path(f"{drive.DeviceID}/"))

    return drives


def copy_file(log_file):
    try:
        shutil.copy2(log_file, Path.cwd())
    except OSError as error:
        logger.error(f"Failed to copy {log_file.name}: {error}")
        return
    logger.info(f"Copied {log_file.name} ({log_file.stat().st_size} bytes)")


def download_log_file(log_file, repo):
    copy_file(log_file)
    commit_and_push_log(repo, log_file)

    ds_log = (DRIVERSTATION_LOGS_DIRECTORY / log_file.stem).with_suffix(DRIVERSTATION_FILE_EXTENSION)
    copy_file(ds_log)



def is_competition_log(log_file) -> bool:
    return re.search(r"_[pqe]", log_file.name) is not None


def download_logs(drive_path, repo):
    try:
        for log_file in drive_path.glob(f"**/*.{LOG_FILE_EXTENSION}"):
            if DOWNLOAD_COMPETITION_LOGS and not is_competition_log(log_file):
                continue
            if is_file_downloaded(log_file):
                logger.info(f"Skipped {log_file.name} - already exists")
                continue
            download_log_file(log_file, repo)
    except Exception as error:
        logger.error(f"Error accessing {drive_path}: {error}")


def monitor_drives():
    logger.info(f"Monitoring for USB drives. Saving logs to {LOGS_DIR}")
    previous_drives = set()

    try:
        repo = Repo()
        logger.info("Git repository initialized")
    except Exception as error:
        logger.exception(f"Error initializing git repository: {error}")
        return

    while True:
        current_drives = get_usb_drives()

        new_drives = current_drives - previous_drives
        for drive in new_drives:
            logger.info(f"New drive detected: {drive}")
            download_logs(drive, repo)
            open_latest_log_in_advantage_scope()

        previous_drives = current_drives
        time.sleep(1)


if __name__ == "__main__":
    LOGS_DIR.mkdir(exist_ok=True)
    os.chdir(LOGS_DIR)

    logger.add("logfile.log",
               rotation="100 MB",
               retention="1 week",
               compression="zip",
               format="{time} | {level} | {message}",
               backtrace=True,
               diagnose=True)
    monitor_drives()
