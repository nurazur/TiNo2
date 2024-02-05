// **********************************************************************************
// Copyright nurazur@gmail.com
// **********************************************************************************
// License
// **********************************************************************************
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

// Licence can be viewed at
// http://www.fsf.org/licenses/gpl.txt

// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// *********************************************************************************
#include <Arduino.h>

class PIRModule
{
    public:
        PIRModule(Configuration& C, Stream* S,  void (*U)(void));
        void init(void);

        void triggered(uint8_t event);
        void check_deadtime(void);

        uint16_t pir_counter;
        bool pir_is_off;
        bool pir_enabled;

    private:
        void registerDevice(void);
        void unregister(void);
        Stream *serial;
        Configuration &Cfg;
        void (*isr)(void);
};