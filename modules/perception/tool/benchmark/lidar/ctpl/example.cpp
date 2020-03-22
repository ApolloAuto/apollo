#include <ctpl.h>
#include <iostream>
#include <string>



void first(int id) {
    std::cout << "hello from " << id << ", function\n";
}

void aga(int id, int par) {
    std::cout << "hello from " << id << ", function with parameter " << par <<'\n';
}

struct Third {
    Third(int v) { this->v = v; std::cout << "Third ctor " << this->v << '\n'; }
    Third(Third && c) { this->v = c.v; std::cout<<"Third move ctor\n"; }
    Third(const Third & c) { this->v = c.v; std::cout<<"Third copy ctor\n"; }
    ~Third() { std::cout << "Third dtor\n"; }
    int v;
};

void mmm(int id, const std::string & s) {
    std::cout << "mmm function " << id << ' ' << s << '\n';
}

void ugu(int id, Third & t) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    std::cout << "hello from " << id << ", function with parameter Third " << t.v <<'\n';
}

int main(int argc, char **argv) {
    ctpl::thread_pool p(2 /* two threads in the pool */);

    std::future<void> qw = p.push(std::ref(first));  // function
    p.push(first);  // function
    p.push(aga, 7);  // function

    {
        struct Second {
            Second(const std::string & cs) : s(cs) { std::cout << "Second ctor\n"; }
            Second(Second && c) : s(std::move(c.s)) { std::cout << "Second move ctor\n"; }
            Second(const Second & c) : s(c.s) { std::cout << "Second copy ctor\n"; };
            ~Second() { std::cout << "Second dtor\n"; }
            void operator()(int id) const {
                std::cout << "hello from " << id << ' ' << this->s << '\n';
            }
        private:
            std::string s;
        } second(", functor");

        p.push(std::ref(second));  // functor, reference
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        p.push(const_cast<const Second &>(second));  // functor, copy ctor
        p.push(std::move(second));  // functor, move ctor
        p.push(second);  // functor, move ctor
        p.push(Second(", functor"));  // functor, move ctor
    }
        {
            Third t(100);

            p.push(ugu, std::ref(t));  // function. reference
            p.push(ugu, t);  // function. copy ctor, move ctor
            p.push(ugu, std::move(t));  // function. move ctor, move ctor

        }
        p.push(ugu, Third(200));  // function



    std::string s = ", lambda";
    p.push([s](int id){  // lambda
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        std::cout << "hello from " << id << ' ' << s << '\n';
    });

    p.push([s](int id){  // lambda
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        std::cout << "hello from " << id << ' ' << s << '\n';
    });

    p.push(mmm, "worked");

    auto f = p.pop();
    if (f) {
        std::cout << "poped function from the pool ";
        f(0);
    }
    // change the number of treads in the pool

    p.resize(1);

    std::string s2 = "result";
    auto f1 = p.push([s2](int){
        return s2;
    });
    // other code here
    //...
    std::cout << "returned " << f1.get() << '\n';

    auto f2 = p.push([](int){
        throw std::exception();
    });
    // other code here
    //...
    try {
        f2.get();
    }
    catch (std::exception & e) {
        std::cout << "caught exception\n";
    }

    // get thread 0
    auto & th = p.get_thread(0);

    return 0;
}
