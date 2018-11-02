#pragma once
#ifndef COMMON_SOURCE_CPP_THREADSAFE_QUEUE_H_
#define COMMON_SOURCE_CPP_THREADSAFE_QUEUE_H_
#include <queue>
#include <mutex>
#include <condition_variable>
#include <initializer_list>

/*
* �̰߳�ȫ����
* TΪ����Ԫ������
* ��Ϊ��std::mutex��std::condition_variable���Ա,���Դ��಻֧�ָ��ƹ��캯��Ҳ��֧�ָ�ֵ������(=)
* */

/* ������֧��c++11�������Է���linux��gcc4.8.5֧�֣�4.4.7��֧�� */
//#define COMPILER_SUPPORTS_CPP11

#define THREADSAFE_QUEUE_QUENTITY_MAX  10000

template<typename T>
class threadsafe_queue {
private:

	//������󻺴�����
	unsigned int quentity_max;

	// data_queue�����ź���
	mutable std::mutex mut;
	mutable std::condition_variable data_cond;
#ifdef COMPILER_SUPPORTS_CPP11
	using queue_type = std::queue<T>;
#else
	typedef  std::queue<T> queue_type;
#endif
	queue_type data_queue;
public:
#ifdef COMPILER_SUPPORTS_CPP11
	using value_type = typename queue_type::value_type;
	using container_type = typename queue_type::container_type;
#else
	typedef typename queue_type::value_type value_type;
	typedef typename queue_type::container_type container_type;
#endif
	threadsafe_queue() {
		quentity_max = THREADSAFE_QUEUE_QUENTITY_MAX;
	}

	threadsafe_queue(const threadsafe_queue&) = delete;
	threadsafe_queue& operator=(const threadsafe_queue&) = delete;

	/*
	* ʹ�õ�����Ϊ�����Ĺ��캯��,����������������
	* */
	template<typename _InputIterator>
	threadsafe_queue(_InputIterator first, _InputIterator last) {
		for (auto itor = first; itor != last; ++itor) {
			data_queue.push(*itor);
		}
	}

	explicit threadsafe_queue(const container_type &c) :data_queue(c) {}
	/*
	* ʹ�ó�ʼ���б�Ϊ�����Ĺ��캯��
	* */
	threadsafe_queue(std::initializer_list<value_type> list) :threadsafe_queue(list.begin(), list.end()) {
	}

	void init(unsigned int quentity) {
		quentity_max = quentity;
	}

	/*
	* ��Ԫ�ؼ�����У��������������������false
	* */
	bool push(const value_type &new_value) {
		std::lock_guard<std::mutex>lk(mut);
		if (data_queue.size() >= quentity_max)
			return false;

		data_queue.push(std::move(new_value));
		data_cond.notify_one();

		return true;
	}

	/*
	* ��Ԫ�ؼ�����У��������������������ָ������ʱ�����Ƴ�һ��Ԫ�������
	* */
	int push(const value_type &new_value, value_type &old_value, unsigned int _quentity = THREADSAFE_QUEUE_QUENTITY_MAX) {
		int ret = 1;
		std::lock_guard<std::mutex>lk(mut);

		unsigned int quentity = _quentity < quentity_max ? _quentity : quentity_max;
		if (data_queue.size() >= quentity && quentity > 0)
		{
			old_value = std::move(data_queue.front());
			data_queue.pop();
			ret = 0;
		}

		data_queue.push(std::move(new_value));
		data_cond.notify_one();
		return ret;
	}

#ifdef COMPILER_SUPPORTS_CPP11
	/*
	* �Ӷ����е���һ��Ԫ��,�������Ϊ�վ�����
	* */
	value_type wait_and_pop() {
		std::unique_lock<std::mutex>lk(mut);
		data_cond.wait(lk, [this] {return !this->data_queue.empty(); });
		auto value = std::move(data_queue.front());
		data_queue.pop();
		return value;
	}
#endif
	/*
	* �Ӷ����е���һ��Ԫ��,�������Ϊ�շ���false
	* */
	bool try_pop(value_type& value) {
		std::lock_guard<std::mutex>lk(mut);
		if (data_queue.empty())
			return false;
		value = std::move(data_queue.front());
		data_queue.pop();
		return true;
	}

	/*
	* �Ӷ�����ȡһ��Ԫ��,��������,�������Ϊ�շ���false
	* */
	bool try_front(value_type& value) {

		std::lock_guard<std::mutex>lk(mut);

		if (data_queue.empty())

			return false;

		value = std::move(data_queue.front());

		return true;

	}

	/*
	* ���ض����Ƿ�Ϊ��
	* */
	auto empty() const->decltype(data_queue.empty()) {
		std::lock_guard<std::mutex>lk(mut);
		return data_queue.empty();
	}
	/*
	* ���ض�����Ԫ������
	* */
	auto size() const->decltype(data_queue.size()) {
		std::lock_guard<std::mutex>lk(mut);
		return data_queue.size();
	}
}; /* threadsafe_queue */

#endif /* COMMON_SOURCE_CPP_THREADSAFE_QUEUE_H_ */
