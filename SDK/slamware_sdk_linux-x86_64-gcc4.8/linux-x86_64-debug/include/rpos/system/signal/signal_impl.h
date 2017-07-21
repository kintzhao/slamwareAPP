#pragma once

#define RPOS_SIGNAL_PARAM(N) RPOS_SIGNAL_PARAM_ ## N
#define RPOS_SIGNAL_ARG(N) RPOS_SIGNAL_ARG_ ## N
#define RPOS_SIGNAL_TEMPLATE_PARAM(N) RPOS_SIGNAL_TEMPLATE_PARAM_ ## N
#define RPOS_SIGNAL_SIGNATURE(N) RPOS_SIGNAL_SIGNATURE_ ## N

#define RPOS_SIGNAL_PARAM_0
#define RPOS_SIGNAL_PARAM_1 T0 a0
#define RPOS_SIGNAL_PARAM_2 T0 a0, T1 a1
#define RPOS_SIGNAL_PARAM_3 T0 a0, T1 a1, T2 a2
#define RPOS_SIGNAL_PARAM_4 T0 a0, T1 a1, T2 a2, T3 a3
#define RPOS_SIGNAL_PARAM_5 T0 a0, T1 a1, T2 a2, T3 a3, T4 a4

#define RPOS_SIGNAL_ARG_0
#define RPOS_SIGNAL_ARG_1 a0
#define RPOS_SIGNAL_ARG_2 a0, a1
#define RPOS_SIGNAL_ARG_3 a0, a1, a2
#define RPOS_SIGNAL_ARG_4 a0, a1, a2, a3
#define RPOS_SIGNAL_ARG_5 a0, a1, a2, a3, a4

#define RPOS_SIGNAL_TEMPLATE_PARAM_0
#define RPOS_SIGNAL_TEMPLATE_PARAM_1 class T0
#define RPOS_SIGNAL_TEMPLATE_PARAM_2 class T0, class T1
#define RPOS_SIGNAL_TEMPLATE_PARAM_3 class T0, class T1, class T2
#define RPOS_SIGNAL_TEMPLATE_PARAM_4 class T0, class T1, class T2, class T3
#define RPOS_SIGNAL_TEMPLATE_PARAM_5 class T0, class T1, class T2, class T3, class T4

#define RPOS_SIGNAL_SIGNATURE_0 void(					)
#define RPOS_SIGNAL_SIGNATURE_1 void(T0				)
#define RPOS_SIGNAL_SIGNATURE_2 void(T0, T1			)
#define RPOS_SIGNAL_SIGNATURE_3 void(T0, T1, T2		)
#define RPOS_SIGNAL_SIGNATURE_4 void(T0, T1, T2, T3	)
#define RPOS_SIGNAL_SIGNATURE_5 void(T0, T1, T2, T3, T4)


#ifdef __GNUC__

#define RPOS_SIGNAL_IMPL(N) \
	template<RPOS_SIGNAL_TEMPLATE_PARAM(N)> \
	class Signal<RPOS_SIGNAL_SIGNATURE(N)> : public detail::SignalImpl<RPOS_SIGNAL_SIGNATURE(N)>{ \
	public: \
		void operator() (RPOS_SIGNAL_PARAM(N)) { \
			typename detail::SignalImpl<RPOS_SIGNAL_SIGNATURE(N)>::SnapshotScope scope(*this); \
			for (auto it = this->handlers_.begin(); it != this->handlers_.end(); it++) { \
				it->second(RPOS_SIGNAL_ARG(N)); \
			} \
		} \
	};

#else

#define RPOS_SIGNAL_IMPL(N) \
	template<RPOS_SIGNAL_TEMPLATE_PARAM(N)> \
	class Signal<RPOS_SIGNAL_SIGNATURE(N)> : public detail::SignalImpl<RPOS_SIGNAL_SIGNATURE(N)>{ \
	public: \
		void operator() (RPOS_SIGNAL_PARAM(N)) { \
			SnapshotScope scope(*this); \
			for (auto it = this->handlers_.begin(); it != this->handlers_.end(); it++) { \
				it->second(RPOS_SIGNAL_ARG(N)); \
			} \
		} \
	};

#endif

