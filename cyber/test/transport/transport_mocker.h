//#include <gmock/gmock.h>
#include <sys/shm.h>

namespace apollo {
namespace cyber {
namespace transport {
/*
class TransportMock {
  TransportMock();
  ~TransportMock();
  MOCK_CONST_METHOD1(shmget,
                     int(key_t key, size_t size, int shmflg));
  MOCK_CONST_METHOD2(shmat,
                     void*(int shmid, const void *shmaddr, int shmflg));
  MOCK_CONST_METHOD3(shmctl, int(int shmid, int cmd, struct shmid_ds *buf));
  MOCK_CONST_METHOD3(shmdt, int(int shmid, int cmd, struct shmid_ds *buf));
};

std::function<int(key_t key, size_t size, int shmflg)> _shmget;
std::function<void*(int shmid, const void *shmaddr, int shmflg)> _shmat;
std::function<int(int shmid, int cmd, struct shmid_ds *buf)> _shmctl;
std::function<int(int shmid, int cmd, struct shmid_ds *buf)> _shmdt;

TransportMock::TransportMock() {
  _shmget = [this](key_t key, size_t size, int shmflg) {
    return shmget(key, size, shmflg);
  };

  _shmat = [this](int shmid, const void *shmaddr, int shmflg) {
    return shmat(shmid, shmaddr, shmflg);
  };

  _shmctl = [this](int shmid, int cmd, struct shmid_ds *buf) {
    return shmctl(shmid, cmd, buf);
  };

  _shmdt = [this](int shmid, int cmd, struct shmid_ds *buf) {
    return shmdt(shmid, cmd, buf);
  };
}

TransportMokc::~TransportMock() {
  _shmget = {};
  _shmat = {};
  _shmctl = {};
  _shmdt = {};
}

int shmget(key_t key, size_t size, int shmflg) {
  return _shmget(key, size, shmflg);
}

void* shmat(int shmid, const void *shmaddr, int shmflg) {
  return _shmat(shmid, shmaddr, shmflg);
}

int shmctl(int shmid, int cmd, struct shmid_ds *buf) {
  return _shmctl(shmid, cmd, buf);
}

int shmdt(int shmid, int cmd, struct shmid_ds *buf) {
  return _shmdt(shmid, cmd, buf);
}*/

}  // namespace transport
}  // namespace cyber
}  // namespace apollo